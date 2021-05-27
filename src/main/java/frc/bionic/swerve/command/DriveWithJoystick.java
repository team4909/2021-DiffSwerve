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

public class DriveWithJoystick extends CommandBase {

  private final AbstractDrivetrain drivetrain;
  private final Joystick joystick;
  
  private NetworkTableEntry sb_enable_joystick;
  private NetworkTableEntry sb_lf_heading, sb_lf_speed;
  private NetworkTableEntry sb_rf_heading, sb_rf_speed;
  private NetworkTableEntry sb_navx_reset, sb_override, sb_use_slew;
  private NetworkTableEntry sb_xSpeed, sb_ySpeed, sb_zSpeed;
  private NetworkTableEntry sb_lr_heading, sb_lr_speed;
  private NetworkTableEntry sb_rr_heading, sb_rr_speed;

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
 
    if (sb_use_slew.getBoolean(true)) {
      xSpeed = xspeedLimiter.calculate(xSpeed);
      ySpeed = yspeedLimiter.calculate(ySpeed);
      rot    = rotLimiter.calculate(rot);
    }

    if (sb_override.getBoolean(false)) {
      xSpeed = sb_xSpeed.getDouble(0);
      ySpeed = sb_ySpeed.getDouble(0);
      rot = sb_zSpeed.getDouble(0);
    } else {
      // when not overriding, show the joystick values on the dashboard
      sb_xSpeed.setDouble(xSpeed);
      sb_ySpeed.setDouble(ySpeed);
      sb_zSpeed.setDouble(rot);
    }

    if (sb_navx_reset.getBoolean(false)) {
      sb_navx_reset.setBoolean(false); // make this a momentary button
      drivetrain.resetGyroAngle();
    }

    // Calls the drive method in Drivetrain. The rotation is only allowed when Button 2 is pressed
    if (sb_enable_joystick.getBoolean(false))
    {
      drivetrain.drive(xSpeed, ySpeed, joystick.getRawButton(2) ? rot : 0);
    }

    var states = drivetrain.getSwerveModuleStates();

    // Update display
    // RF, LF, LR, RR
    sb_rf_heading.setDouble(states[0].angle.getDegrees());
    sb_rf_speed.setDouble(states[0].speedMetersPerSecond);

    sb_lf_heading.setDouble(states[1].angle.getDegrees());
    sb_lf_speed.setDouble(states[1].speedMetersPerSecond);

    sb_lr_heading.setDouble(states[2].angle.getDegrees());
    sb_lr_speed.setDouble(states[2].speedMetersPerSecond);
    
    sb_rr_heading.setDouble(states[3].angle.getDegrees());
    sb_rr_speed.setDouble(states[3].speedMetersPerSecond);
  }

  protected void initShuffleboard(){
   
    ShuffleboardTab tab = Shuffleboard.getTab("Maintenance");

    sb_enable_joystick = tab.addPersistent("Joystick Enabled", true)
                            .withSize(2, 2)
                            .withWidget(BuiltInWidgets.kToggleButton).getEntry();

    ShuffleboardTab dtTab = Shuffleboard.getTab("Drivetrain");
    int row = 0;
    dtTab.getLayout("Front", BuiltInLayouts.kGrid).withSize(10, 1).withPosition(0, row);
    
    row = 1;
    sb_lf_heading = dtTab.add("LF Heading", 0).withSize(2, 2).withPosition(0, row).getEntry();
    sb_lf_speed   = dtTab.add("LF Speed",   0).withSize(2, 2).withPosition(2, row).getEntry();

    sb_rf_heading = dtTab.add("RF Heading", 0).withSize(2, 2).withPosition(6, row).getEntry();
    sb_rf_speed   = dtTab.add("RF Speed",   0).withSize(2, 2).withPosition(8, row).getEntry();
    
    row = 3;
    sb_navx_reset = dtTab.add("NavX Reset", false).withSize(2, 2).withPosition(0, row).withWidget(BuiltInWidgets.kToggleButton).getEntry();
    sb_override   = dtTab.add("Use Overrides",   false).withSize(2, 2).withPosition(4, row).withWidget(BuiltInWidgets.kToggleButton).getEntry();
    sb_use_slew   = dtTab.add("Enable Slew", true).withSize(2, 2).withPosition(8, row).withWidget(BuiltInWidgets.kToggleButton).getEntry();
    
    row = 5;
    sb_xSpeed     = dtTab.add("xSpeed Override", 0).withSize(3, 2).withPosition(0, row).withWidget(BuiltInWidgets.kNumberSlider).getEntry();
    sb_ySpeed     = dtTab.add("ySpeed Override", 0).withSize(4, 2).withPosition(3, row).withWidget(BuiltInWidgets.kNumberSlider).getEntry();
    sb_zSpeed     = dtTab.add("zSpeed Override", 0).withSize(3, 2).withPosition(7, row).withWidget(BuiltInWidgets.kNumberSlider).getEntry();
    
    row = 7;
    sb_lr_heading = dtTab.add("LR Heading", 0).withSize(2, 2).withPosition(0, row).getEntry();
    sb_lr_speed   = dtTab.add("LR Speed",   0).withSize(2, 2).withPosition(2, row).getEntry();
    
    sb_rr_heading = dtTab.add("RR Heading", 0).withSize(2, 2).withPosition(6, row).getEntry();
    sb_rr_speed   = dtTab.add("RR Speed",   0).withSize(2, 2).withPosition(8, row).getEntry();
  }
}
