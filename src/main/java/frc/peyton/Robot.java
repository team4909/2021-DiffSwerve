/*
 * Team 4909, Bionics
 * Billerica Memorial High School
 *
 * Copyright:
 *   2021 Bionics
 *
 * License:
 *   MIT: https://opensource.org/licenses/MIT
 *   See the LICENSE file in the project's top-level directory for details.
 */

package frc.peyton;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.SlewRateLimiter;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class Robot extends TimedRobot {
  private final Joystick m_controller = new Joystick(0);
  private final Drivetrain m_swerve = new Drivetrain();

  // Slew rate limiters to make joystick inputs more gentle; 1/3 sec from 0 to 1.
  private final SlewRateLimiter m_xspeedLimiter = new SlewRateLimiter(3);
  private final SlewRateLimiter m_yspeedLimiter = new SlewRateLimiter(3);
  private final SlewRateLimiter m_rotLimiter = new SlewRateLimiter(3);

  @Override
  public void robotInit() {
    // NetworkTable table = NetworkTableInstance.getDefault().getTable("MyTable");
    // table.getEntry("")
    SmartDashboard.putBoolean("Override", false);
    SmartDashboard.putBoolean("ZERO", false);
    SmartDashboard.putBoolean("Enable Slew", true);
  }

  @Override
  public void robotPeriodic() {
    m_swerve.periodic();
  }

  @Override
  public void teleopInit() {
  }

  // @Override
  // public void autonomousPeriodic() {
  //   driveWithJoystick(false);
  //   m_swerve.updateOdometry();
  // }

  @Override
  public void teleopPeriodic() {
    driveWithJoystick(true);
  }

  private void driveWithJoystick(boolean fieldRelative) {
    // Get the x speed. We are inverting this because Xbox controllers return
    // negative values when we push forward.
    double xSpeed = -m_controller.getY();
        // -m_xspeedLimiter.calculate(m_controller.getY(GenericHID.Hand.kLeft))
        //     * frc.robot.Drivetrain.kMaxSpeed;

    // Get the y speed or sideways/strafe speed. We are inverting this because
    // we want a positive value when we pull to the left. Xbox controllers
    // return positive values when you pull to the right by default.
    double ySpeed = -m_controller.getX();
        // -m_yspeedLimiter.calculate(m_controller.getX(GenericHID.Hand.kLeft))
        //     * frc.robot.Drivetrain.kMaxSpeed;

    // Get the rate of angular rotation. We are inverting this because we want a
    // positive value when we pull to the left (remember, CCW is positive in
    // mathematics). Xbox controllers return positive values when you pull to
    // the right by default.
    double rot = -m_controller.getZ();
        // -m_rotLimiter.calculate()
        //     * frc.robot.Drivetrain.kMaxAngularSpeed;

    if (Math.abs(rot) < .1) {
      rot = 0;
    }
    if (Math.abs(xSpeed) < .1) {
      xSpeed = 0;
    }
    if (Math.abs(ySpeed) < .1) {
      ySpeed = 0;
    }

    

    if (SmartDashboard.getBoolean("Override", false)) {
      xSpeed = SmartDashboard.getNumber("xSpeed", 0);
      ySpeed = SmartDashboard.getNumber("ySpeed", 0);
      rot    = SmartDashboard.getNumber("rot",    0);
    }

    if (SmartDashboard.getBoolean("ZERO", false)) {
      SmartDashboard.putBoolean("ZERO", false);
      xSpeed = 0;
      ySpeed = 0;
      rot    = 0;
    }

    if (SmartDashboard.getBoolean("Enable Slew", false)) {
      
      xSpeed = m_xspeedLimiter.calculate(xSpeed);
      ySpeed = m_yspeedLimiter.calculate(ySpeed);
      rot    = m_rotLimiter.calculate(rot);
    }

    SmartDashboard.putNumber("xSpeed", xSpeed);
    SmartDashboard.putNumber("ySpeed", ySpeed);
    SmartDashboard.putNumber("rot",    rot);

    m_swerve.drive(xSpeed, ySpeed, rot);
  }
}
