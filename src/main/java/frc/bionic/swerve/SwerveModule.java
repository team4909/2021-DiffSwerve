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

package frc.bionic.swerve;

import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.wpilibj.geometry.Rotation2d;
import edu.wpi.first.wpilibj.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInLayouts;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardLayout;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;


public class SwerveModule
{
  // Gear ratio of first two pairs of gears.
  // Yaw does not depend on the third pair.
  private double             GEAR_RATIO_12;

  // Gear ratio of all three pairs of gears.
  // Wheel speed depends on all three.
  private double             GEAR_RATIO_123;

  // Maximum yaw speed in RPM
  private double            MAX_YAW_SPEED_RPM;

  // Wheel diameter in meters
  private double            WHEEL_DIAMETER_METERS;

  // Desired state -- velocity of wheel in RPM, angle in degrees
  // Angles are measured counter-clockwise, with zero being "robot forward"
  public double             desiredWheelSpeedRPM;
  public double             desiredYawDegrees;
  
  // Devices, Sensors, Actuators
  MotorCANSparkMaxNeo       m_driveMotorA;
  MotorCANSparkMaxNeo       m_driveMotorB;
  YawEncoderGrayhill63R128  m_yawEncoder;
  
  // Shuffleboard-related
  private String            name;
  private String            shuffleboardTabName;
  private NetworkTableEntry sb_yaw;
  private NetworkTableEntry sb_wheel_rpm;
  private NetworkTableEntry sb_wheel_calc;
  private NetworkTableEntry sb_yaw_calc;
  private NetworkTableEntry sb_trans;

  
  


  public SwerveModule(double gearRatio12, double gearRatio123, double maxYawSpeedRpm,
                      int driveMotorChannelA, int driveMotorChannelB,
                      int dioEncoderChannelA, int dioEncoderChannelB,
                      double wheelDiameterMeters,
                      String name, String shuffleboardTabName)
  {
    // Save arguments
    GEAR_RATIO_12             = gearRatio12;
    GEAR_RATIO_123            = gearRatio123;
    MAX_YAW_SPEED_RPM         = maxYawSpeedRpm;
    WHEEL_DIAMETER_METERS     = wheelDiameterMeters;
    this.name                 = name;
    this.shuffleboardTabName  = shuffleboardTabName;

    // Instantiate the two swerve motors our swerve module incorporates
    m_driveMotorA =
      new MotorCANSparkMaxNeo(driveMotorChannelA, name + " A", shuffleboardTabName);
    m_driveMotorB =
      new MotorCANSparkMaxNeo(driveMotorChannelB, name + " B", shuffleboardTabName);

    // Instantiate the yaw encoder our swerve module incorporates
    m_yawEncoder =
      new YawEncoderGrayhill63R128(dioEncoderChannelA, dioEncoderChannelB, "Grayhill", shuffleboardTabName);

    // If we're using a relative encoder, assume the robot starts up
    // facing to the zero position. If we're using an absolute
    // encoder, the `setZero` method is a no-op.
    m_yawEncoder.setZero();

    // Initialize all of the shuffleboard inputs and outputs
    initShuffleboard();
  }

  /**
   * Reset module's encoder to angle zero
   */
  public void setZero()
  {
    m_yawEncoder.setZero();
  }

  /**
   * Set the desired module state. Desired wheel speed and angle are converted
   * from meters per second and Rotation2d, to revolutions per minute and
   * degrees.
   *
   * @param state
   *   The desired module state, with speed in units of meters per second, and
   *   angle in units of radians.
   */
  public void setModuleState(SwerveModuleState state)
  {
    // Convert the state to our internally-used units, and save
    desiredWheelSpeedRPM = frc.bionic.Conversion.mpsToRpm(state.speedMetersPerSecond, WHEEL_DIAMETER_METERS);
    desiredYawDegrees = state.angle.getDegrees();

    // Keep shuffleboard up to date with provided state
    sb_yaw.setDouble(desiredYawDegrees);
    sb_wheel_rpm.setDouble(desiredWheelSpeedRPM);
  }

  /**
   * Retrieve the current module state
   */
  public SwerveModuleState getModuleState()
  {
    double speedMPS = frc.bionic.Conversion.rpmToMps(desiredWheelSpeedRPM, WHEEL_DIAMETER_METERS);
    Rotation2d yawRotation = Rotation2d.fromDegrees(desiredYawDegrees);

    return new SwerveModuleState(speedMPS, yawRotation);
  }

  /**
   * Function that should be called periodically, typically from
   * `robotPeriodic` or from a Command's `execute` method.
   */
  public void periodic()
  {
    double                    calculatedYawRPM;

    // Get desired yaw and wheel speed
    desiredYawDegrees = sb_yaw.getDouble(0);
    desiredWheelSpeedRPM = sb_wheel_rpm.getDouble(0);

    // Determine the percentage of output, based on difference between
    // yaw goal and actual angle, to be used in the RPM calculation.
    calculatedYawRPM = m_yawEncoder.getOutputSignedPercent(desiredYawDegrees) * MAX_YAW_SPEED_RPM;

    // report to dashboard
    sb_wheel_calc.setDouble(desiredWheelSpeedRPM);
    sb_yaw_calc.setDouble(calculatedYawRPM);

    // Given the desired wheel and yaw RPM, calculate and specify the
    // motor speeds necessary to achieve them
    setMotorSpeedsRPM(desiredWheelSpeedRPM, calculatedYawRPM);

    // Ensure that our motor and encoder periodic functions are called, too
    m_driveMotorA.periodic();
    m_driveMotorB.periodic();
    m_yawEncoder.periodic();

    syncShuffleboard();
  }

  /**
   * Zero the yaw encoder
   */
  public void setZeroYaw()
  {
    m_yawEncoder.setZero();
  }

  /**
   * Get wheel speed in RPM
   *
   * @param aMotorRPM
   *   The speed of the first motor, in RPM
   *
   * @param bMotorRPM
   *   The speed of the second motor, in RPM
   *
   * @return
   *   The wheel speed, in RPM
   */
  protected double motorRPMsToWheelRPM(double aMotorRPM, double bMotorRPM)
  {
    // Translation is calculated as half the difference of a and b,
    // adjusted by gear ratio. Translation is dependent on all three
    // pairs of gears. The differential pinion generates translation 
    // as a function of the speed of the top and bottom differential gears.
    return ((aMotorRPM - bMotorRPM) / 2) * GEAR_RATIO_123 ;
  }

  /**
   * Get module yaw in RPM
   *
   * @param aMotorRPM
   *   The speed of the first motor, in RPM
   *
   * @param bMotorRPM
   *   The speed of the second motor, in RPM
   *
   * @return
   *   The speed rotation of the module, in RPM
   */
  protected double motorRPMsToModuleYawRPM(double aMotorRPM, double bMotorRPM)
  {
    // Yaw is calculated as the average of a and b, adjusted by gear ratio
    // Yaw does not depend on the third pair of gears, thus GEAR_RATIO_12
    // only includes the first two gear reductions.
    return ((aMotorRPM + bMotorRPM) / 2) * GEAR_RATIO_12;
  }

  // convert desired translation and yaw RPMs to motor RPMs
  protected void setMotorSpeedsRPM(double wheelRPM, double yawRPM)
  {
    double aRPM = (yawRPM / GEAR_RATIO_12) + (wheelRPM / GEAR_RATIO_123);
    double bRPM = (yawRPM / GEAR_RATIO_12) - (wheelRPM / GEAR_RATIO_123);

    m_driveMotorA.setGoalRPM(aRPM);
    m_driveMotorB.setGoalRPM(bRPM);
  }

  /**
   * Initialize the shuffleboard interface for this motor
   */
  protected void initShuffleboard()
  {
    ShuffleboardTab           tab;
    ShuffleboardLayout        layout;

    tab            = Shuffleboard.getTab(shuffleboardTabName);
    layout         = tab.getLayout("Module " + name, BuiltInLayouts.kList);

    sb_trans       = layout.add("Translation",  0).getEntry();
  }

  /**
   * Update dynamic values on shuffleboard, and read values and reset based on
   * read values, any settable parameters.
   */
  void syncShuffleboard()
  {
    double                    aMotorRPM;
    double                    bMotorRPM;
    double                    currentWheelRPM;

    // get the motor speeds
    aMotorRPM = m_driveMotorA.getVelocityRPM();
    bMotorRPM = m_driveMotorB.getVelocityRPM();

    // calculate the wheel speed from those motor speeds
    currentWheelRPM = motorRPMsToWheelRPM(aMotorRPM, bMotorRPM);

    // report to shuffleboard
    sb_trans.setDouble(currentWheelRPM);
  }
}
