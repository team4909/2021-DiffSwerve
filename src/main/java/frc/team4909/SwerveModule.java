package frc.team4909;

import frc.bionic.Conversion;
import frc.bionic.swerve.*;

public class SwerveModule extends AbstractSwerveModule{

  /**
  Gear ratio of first two pairs of gears.
  Yaw does not depend on the third pair.
  */

  // Gear ratio for yaw
  private static final double       GEAR_RATIO_YAW          =  (13.0 / 82.0);

  // Gear ratio of wheel rotation
  private static final double       GEAR_RATIO_WHEEL_SPEED  =  (13.0 / 82.0) * (45.0 / 15.0) * (28.0 / 36.0);

  // Maximum yaw speed in RPM
  private static final double       MAX_YAW_SPEED_RPM       = 200;

  // The wheel diameter, in meters
  public static final double      WHEEL_DIAMETER_METERS    = Conversion.inchesToMeters(3);

  public SwerveModule(int canDeviceChannelA, int canDeviceChannelB,
                              int dioEncoderChannel,
                              String name, String shuffleboardTabName)
  {
    super(name, shuffleboardTabName);

    // Instantiate the two swerve motors our swerve module incorporates
    // true makes the motor go clockwise, if both motors are going clockwise, then positve values = translation
    IMotor motorA = new MotorFalcon500(canDeviceChannelA, true, name + " A", shuffleboardTabName);
    IMotor motorB = new MotorFalcon500(canDeviceChannelB, true,  name + " B", shuffleboardTabName);

    // Instantiate the yaw encoder our swerve module incorporates
    IYawEncoder encoder = new YawEncoderRevHex(dioEncoderChannel, name, shuffleboardTabName);

    // Now we have everything our superclass needs to do its thing. Do it!
    initialize(GEAR_RATIO_YAW, GEAR_RATIO_WHEEL_SPEED, MAX_YAW_SPEED_RPM, 
               motorA, motorB, encoder, WHEEL_DIAMETER_METERS);
  }

  // abstract method implementation
  protected double getARPM(double wheelRPM, double yawRPM)
  {
    return (wheelRPM / GEAR_RATIO_WHEEL_SPEED) + (yawRPM / GEAR_RATIO_YAW);
  }

  // abstract method implementation
  protected double getBRPM(double wheelRPM, double yawRPM)
  {
    return (wheelRPM / GEAR_RATIO_WHEEL_SPEED) - (yawRPM / GEAR_RATIO_YAW);
  }
}
