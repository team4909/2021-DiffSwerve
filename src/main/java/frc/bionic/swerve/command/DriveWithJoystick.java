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

import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.SlewRateLimiter;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.peyton.Drivetrain;

public class DriveWithJoystick extends CommandBase {

  private final Drivetrain drivetrain;
  private final Joystick joystick;
  private final String name = "Maintenance";
  private NetworkTableEntry sb_enable_joystick_drive;

  // Joystick returns [-1.0, 1.0] with 0 being the center point on
  // each axis. Provide a small dead zone so that small hand movements
  // when the joystick is centered are ignored
  private final double JOYSTICK_DEADZONE = 0.15;


  // Slew rate limiters to make joystick inputs more gentle; 1/3 sec from 0 to 1.
  private final SlewRateLimiter xspeedLimiter = new SlewRateLimiter(3);
  private final SlewRateLimiter yspeedLimiter = new SlewRateLimiter(3);
  private final SlewRateLimiter rotLimiter    = new SlewRateLimiter(3);

  public DriveWithJoystick(Drivetrain drivetrain, Joystick joystick) {
    this.drivetrain = drivetrain;
    this.joystick = joystick;
    addRequirements(drivetrain);

    initShuffleboard();
    SmartDashboard.putNumber("xSpeed", 0);
    SmartDashboard.putNumber("ySpeed", 0);
    SmartDashboard.putNumber("rot",    0);
  }

  public void execute() {
    // Get the x speed. We are inverting this because Xbox controllers return
    // negative values when we push forward.
    double xSpeed = joystick.getY();

    // Get the y speed or sideways/strafe speed. We are inverting this because
    // we want a positive value when we pull to the left. Xbox controllers
    // return positive values when you pull to the right by default.

    double ySpeed = joystick.getX();

    // Get the rate of angular rotation. We are inverting this because we want a
    // positive value when we pull to the left (remember, CCW is positive in
    // mathematics). Xbox controllers return positive values when you pull to
    // the right by default.
    double rot = joystick.getZ();

    if (Math.abs(xSpeed) < JOYSTICK_DEADZONE) {
      xSpeed = 0;
    }
    if (Math.abs(ySpeed) < JOYSTICK_DEADZONE) {
      ySpeed = 0;
    }
 

    if (SmartDashboard.getBoolean("Enable Slew", false)) {
      xSpeed = xspeedLimiter.calculate(xSpeed);
      ySpeed = yspeedLimiter.calculate(ySpeed);
      rot    = rotLimiter.calculate(rot);
    }

    SmartDashboard.putNumber("xSpeed", xSpeed);
    SmartDashboard.putNumber("ySpeed", ySpeed);
    SmartDashboard.putNumber("rot",    rot);

    // Calls the drive method in Drivetrain. The rotation is only allowed when Button 2 is pressed
    if (sb_enable_joystick_drive.getBoolean(true))
    {
      drivetrain.drive(xSpeed, ySpeed, joystick.getRawButton(2) ? rot : 0);
    }
  }
  protected void initShuffleboard(){
    ShuffleboardTab           tab;

    tab = Shuffleboard.getTab(name);

    sb_enable_joystick_drive = tab.add("joystick enabled", true).getEntry();
  }
}
