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

package frc.robot;

import frc.bionic.swerve.AbstractDrivetrain;
import frc.robot.commands.DriveWithJoystick;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.SlewRateLimiter;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;

public class Robot extends TimedRobot {
  private final Joystick        joystickController = new Joystick(0);
  private AbstractDrivetrain    drivetrain;

  // Shuffleboard-related
  NetworkTableEntry         sb_robot_type;


  @Override
  public void robotInit() {
    ShuffleboardTab           tab;

    tab = Shuffleboard.getTab("Robot Setup");
    sb_robot_type = tab.addPersistent("Drivetrain Type",  "Enter 'peyton' or 'team4909'").getEntry();

    SmartDashboard.putBoolean("Override", false);
    SmartDashboard.putBoolean("ZERO", false);
    SmartDashboard.putBoolean("Enable Slew", true);
  }

  @Override
  public void robotPeriodic() {
    String type;
    
    // If we're not yet configured with a drivetrain type...
    if (drivetrain == null) {
      // See if the user has entered a known drivetrain type
      type = sb_robot_type.getString("UNCONFIGURED");
      if (type.equals("peyton")) {
        var defaultCommand = new DriveWithJoystick(joystickController);
        drivetrain = new frc.peyton.Drivetrain(defaultCommand);
      } else if (type.equals("team4909")) {
        drivetrain = new frc.team4909.Drivetrain();
      } else {
        System.out.println("Shuffleboard's 'Robot Selection/Drivetrain Type' is invalid: " + type);
        return;
      }
    }
    drivetrain.periodic();
  }

  @Override
  public void teleopInit() {
  }

  @Override
  public void teleopPeriodic() {
    // driveWithJoystick(true);
  }

  // private void driveWithJoystick(boolean fieldRelative) {
  //   // Get the x speed. We are inverting this because Xbox controllers return
  //   // negative values when we push forward.
  //   double xSpeed = -joystickController.getY();

  //   // Get the y speed or sideways/strafe speed. We are inverting this because
  //   // we want a positive value when we pull to the left. Xbox controllers
  //   // return positive values when you pull to the right by default.
  //   double ySpeed = -joystickController.getX();

  //   // Get the rate of angular rotation. We invert this because we want a
  //   // positive value when we pull to the left (remember, CCW is positive in
  //   // mathematics). Xbox controllers return positive values when you pull to
  //   // the right by default.
  //   double rotate = 0;
    
  //   // only allow turning if thumb button is presses
  //   if (joystickController.getRawButton(2)) {
  //     rotate = -joystickController.getZ();
  //   }

  //   // Implement dead zones on joystick to avoid inadvertent movement
  //   if (Math.abs(rotate) < .5)   rotate = 0;
  //   if (Math.abs(xSpeed) < .2)   xSpeed = 0;
  //   if (Math.abs(ySpeed) < .2)   ySpeed = 0;

  //   // Allow overriding speeds from dashboard
  //   if (SmartDashboard.getBoolean("Override", false)) {
  //     xSpeed = SmartDashboard.getNumber("xSpeed", 0);
  //     ySpeed = SmartDashboard.getNumber("ySpeed", 0);
  //     rotate = SmartDashboard.getNumber("rotate",    0);
  //   }

  //   // Quickly zero all speeds from dashboard
  //   if (SmartDashboard.getBoolean("ZERO", false)) {
  //     SmartDashboard.putBoolean("ZERO", false);
  //     xSpeed = 0;
  //     ySpeed = 0;
  //     rotate = 0;
  //   }

  //   // Use the slew limited on joystick input, if enabled from dashboard
  //   if (SmartDashboard.getBoolean("Enable Slew", false)) {
  //     xSpeed = m_xspeedLimiter.calculate(xSpeed);
  //     ySpeed = m_yspeedLimiter.calculate(ySpeed);
  //     rotate = m_rotLimiter.calculate(rotate);
  //   }

  //   // Log speeds to dashboard
  //   SmartDashboard.putNumber("xSpeed", xSpeed);
  //   SmartDashboard.putNumber("ySpeed", ySpeed);
  //   SmartDashboard.putNumber("rotate", rotate);

  //   // Do it!
  //   drivetrain.drive(xSpeed, ySpeed, rotate);
  // }
}
