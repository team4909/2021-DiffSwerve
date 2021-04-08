package frc.team4909;

import frc.bionic.swerve.*;

public class team4909SwerveModule extends frc.bionic.swerve.SwerveModule{

  /**
  Gear ratio of first two pairs of gears.
  Yaw does not depend on the third pair.
  */

  //TODO Get values
  private static final double       GEAR_RATIO_YAW          =  0; 

  // Gear ratio of all three pairs of gears.
  // Wheel speed depends on all three.
  private static final double       GEAR_RATIO_WHEEL_SPEED  =  0; 

  // Maximum yaw speed in RPM
  private static final double       MAX_YAW_SPEED_RPM       = 0; 

  // The wheel diameter, in meters
  private static final double      WHEEL_DIAMETER_METERS    = 0; 

  public team4909SwerveModule(int canDeviceChannelA, int canDeviceChannelB,
                            int dioEncoderChannelA, int dioEncoderChannelB,
                            String name, String shuffleboardTabName)
  {
    super(name, shuffleboardTabName);

    // Instantiate the two swerve motors our swerve module incorporates
    IMotor motorA = new MotorFaclon500(canDeviceChannelA, name + " A", shuffleboardTabName);
    IMotor motorB = new MotorFaclon500(canDeviceChannelB, name + " B", shuffleboardTabName);

    // Instantiate the yaw encoder our swerve module incorporates
    IYawEncoder encoder;
    if (name == "RR")   // currently, right-rear has RevHex encoder; others have Grayhill
    {
      encoder = new YawEncoderRevHex(dioEncoderChannelA, name, shuffleboardTabName);
    }
    else
    {
      encoder = new YawEncoderGrayhill63R128(dioEncoderChannelA, dioEncoderChannelB, 
                                              name, shuffleboardTabName);
    }

    // Now we have everything our superclass needs to do its thing. Do it!
    initialize(GEAR_RATIO_YAW, GEAR_RATIO_WHEEL_SPEED, MAX_YAW_SPEED_RPM, 
               motorA, motorB, encoder, WHEEL_DIAMETER_METERS);
  }
}