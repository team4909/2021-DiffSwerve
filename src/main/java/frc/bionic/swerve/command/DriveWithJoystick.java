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
import edu.wpi.first.wpilibj.shuffleboard.BuiltInLayouts;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInWidgets;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.bionic.swerve.AbstractDrivetrain;
import frc.robot.Robot;

public class DriveWithJoystick extends CommandBase {

  private final AbstractDrivetrain drivetrain;
  private final Joystick joystick;
  
  private NetworkTableEntry sb_enable_joystick;
  

  // Joystick returns [-1.0, 1.0] with 0 being the center point on
  // each axis. Provide a small dead zone so that small hand movements
  // when the joystick is centered are ignored
  private final double JOYSTICK_DEADZONE = 0.2;


  // Slew rate limiters to make joystick inputs more gentle; 1/3 sec from 0 to 1.
  private final SlewRateLimiter xspeedLimiter = new SlewRateLimiter(3);
  private final SlewRateLimiter yspeedLimiter = new SlewRateLimiter(3);
  private final SlewRateLimiter rotLimiter    = new SlewRateLimiter(3);

  public DriveWithJoystick(AbstractDrivetrain drivetrain, Joystick joystick) {
    this.drivetrain = drivetrain;
    this.joystick = joystick;
    addRequirements(drivetrain);

    initShuffleboard();
  }

  public void execute() {
    System.out.println("DriveWithJS");
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
 
    if (Robot.debugDash.dtTab.useSlew()) {
      xSpeed = xspeedLimiter.calculate(xSpeed);
      ySpeed = yspeedLimiter.calculate(ySpeed);
      rot    = rotLimiter.calculate(rot);
    }

    if (Robot.debugDash.dtTab.useDTOverride()) {
      xSpeed = Robot.debugDash.dtTab.getxSpeedOverride();
      ySpeed = Robot.debugDash.dtTab.getySpeedOverride();
      rot = Robot.debugDash.dtTab.getzSpeedOverride();

      drivetrain.drive(xSpeed, ySpeed, rot);
      return;
    }

    // Calls the drive method in Drivetrain. The rotation is only allowed when Button 2 is pressed
    if (Robot.debugDash.dtTab.isJoystickEnabled())
    {
      drivetrain.drive(xSpeed, ySpeed, joystick.getRawButton(2) ? rot : 0);
    }

    
  }

  protected void initShuffleboard(){
   
    // ShuffleboardTab tab = Shuffleboard.getTab("Maintenance");

    // sb_enable_joystick = tab.addPersistent("Joystick Enabled", true)
    //                         .withSize(2, 2)
    //                         .withWidget(BuiltInWidgets.kToggleButton).getEntry();

    
  }
}
