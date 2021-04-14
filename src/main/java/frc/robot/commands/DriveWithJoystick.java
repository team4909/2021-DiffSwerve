package frc.robot.commands;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.SlewRateLimiter;
import edu.wpi.first.wpilibj.command.Command;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.bionic.swerve.AbstractDrivetrain;

public class DriveWithJoystick extends Command {

  private Joystick js;
  private AbstractDrivetrain drivetrain;

   // Slew rate limiters to make joystick inputs more gentle; 1/3 sec from 0 to 1.
   private final SlewRateLimiter m_xspeedLimiter = new SlewRateLimiter(3);
   private final SlewRateLimiter m_yspeedLimiter = new SlewRateLimiter(3);
   private final SlewRateLimiter m_rotLimiter = new SlewRateLimiter(3);

  public DriveWithJoystick(Joystick js, AbstractDrivetrain drivetrain) {
    super();
    this.js = js;
    this.drivetrain = drivetrain;
  }

  @Override
  protected void execute() {
    // Get the x speed. We are inverting this because Xbox controllers return
    // negative values when we push forward.
    double xSpeed = -js.getY();

    // Get the y speed or sideways/strafe speed. We are inverting this because
    // we want a positive value when we pull to the left. Xbox controllers
    // return positive values when you pull to the right by default.
    double ySpeed = -js.getX();

    // Get the rate of angular rotation. We invert this because we want a
    // positive value when we pull to the left (remember, CCW is positive in
    // mathematics). Xbox controllers return positive values when you pull to
    // the right by default.
    double rotate = 0;
    
    // only allow turning if thumb button is presses
    if (js.getRawButton(2)) {
      rotate = -js.getZ();
    }

    // Implement dead zones on joystick to avoid inadvertent movement
    if (Math.abs(rotate) < .5)   rotate = 0;
    if (Math.abs(xSpeed) < .2)   xSpeed = 0;
    if (Math.abs(ySpeed) < .2)   ySpeed = 0;

    // Allow overriding speeds from dashboard
    if (SmartDashboard.getBoolean("Override", false)) {
      xSpeed = SmartDashboard.getNumber("xSpeed", 0);
      ySpeed = SmartDashboard.getNumber("ySpeed", 0);
      rotate = SmartDashboard.getNumber("rotate",    0);
    }

    // Quickly zero all speeds from dashboard
    if (SmartDashboard.getBoolean("ZERO", false)) {
      SmartDashboard.putBoolean("ZERO", false);
      xSpeed = 0;
      ySpeed = 0;
      rotate = 0;
    }

    // Use the slew limited on joystick input, if enabled from dashboard
    if (SmartDashboard.getBoolean("Enable Slew", false)) {
      xSpeed = m_xspeedLimiter.calculate(xSpeed);
      ySpeed = m_yspeedLimiter.calculate(ySpeed);
      rotate = m_rotLimiter.calculate(rotate);
    }

    // Log speeds to dashboard
    SmartDashboard.putNumber("xSpeed", xSpeed);
    SmartDashboard.putNumber("ySpeed", ySpeed);
    SmartDashboard.putNumber("rotate", rotate);

    // Do it!
    drivetrain.drive(xSpeed, ySpeed, rotate);
    
  }

  @Override
  protected boolean isFinished() {
    // Command is only interrupted, never stopped
    return false;
  }
  
}
