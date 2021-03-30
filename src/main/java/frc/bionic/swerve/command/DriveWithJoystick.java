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

package frc.bionic.swerve.command;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.SlewRateLimiter;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.peyton.Drivetrain;

public class DriveWithJoystick extends CommandBase {

  private final Drivetrain m_subsystem;
  private final Joystick m_controller;

  // Joystick returns [-1.0, 1.0] with 0 being the center point on
  // each axis. Provide a small dead zone so that small hand movements
  // when the joystick is centered are ignored
  private final double JOYSTICK_DEADZONE = 0.1;

  // Slew rate limiters to make joystick inputs more gentle; 1/3 sec from 0 to 1.
  private final SlewRateLimiter m_xspeedLimiter = new SlewRateLimiter(3);
  private final SlewRateLimiter m_yspeedLimiter = new SlewRateLimiter(3);
  private final SlewRateLimiter m_rotLimiter    = new SlewRateLimiter(3);

  public DriveWithJoystick(Drivetrain subsystem, Joystick controller) {
    m_subsystem = subsystem;
    m_controller = controller;
    addRequirements(m_subsystem);

    SmartDashboard.putNumber("xSpeed", 0);
    SmartDashboard.putNumber("ySpeed", 0);
    SmartDashboard.putNumber("rot",    0);
  }

  public void execute() {
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

    if (Math.abs(rot) < JOYSTICK_DEADZONE) {
      rot = 0;
    }
    if (Math.abs(xSpeed) < JOYSTICK_DEADZONE) {
      xSpeed = 0;
    }
    if (Math.abs(ySpeed) < JOYSTICK_DEADZONE) {
      ySpeed = 0;
    }

    

    if (SmartDashboard.getBoolean("Override", false)) {
      xSpeed = SmartDashboard.getNumber("xSpeed", 0);
      ySpeed = SmartDashboard.getNumber("ySpeed", 0);
      rot    = SmartDashboard.getNumber("rot",    0);
    }

    if (SmartDashboard.getBoolean("Enable Slew", false)) {
      xSpeed = m_xspeedLimiter.calculate(xSpeed);
      ySpeed = m_yspeedLimiter.calculate(ySpeed);
      rot    = m_rotLimiter.calculate(rot);
    }

    SmartDashboard.putNumber("xSpeed", xSpeed);
    SmartDashboard.putNumber("ySpeed", ySpeed);
    SmartDashboard.putNumber("rot",    rot);

    m_subsystem.drive(xSpeed, ySpeed, rot);
  }
}
