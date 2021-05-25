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


public abstract class AbstractSwerveModule
{
  // Gear ratio for yaw. This may not include all pairs of gears.
  private double             GEAR_RATIO_YAW;

  // Gear ratio for wheel speed. Typically, this includes all pairs of
  // gears.
  private double             GEAR_RATIO_WHEEL_SPEED;

  // Maximum yaw speed in RPM
  private double            MAX_YAW_SPEED_RPM;

  // Wheel diameter in meters
  private double            WHEEL_DIAMETER_METERS;

  // Desired state -- velocity of wheel in RPM, angle in degrees
  // Angles are measured counter-clockwise, with zero being "robot forward"
  public double             desiredWheelSpeedRPM = 0.0;
  public double             desiredYawDegrees = 0.0;
  
  // Devices, Sensors, Actuators
  IMotor                    motorA;
  IMotor                    motorB;
  IYawEncoder               yawEncoder;
  
  // Shuffleboard-related
  private String            name;
  private String            shuffleboardTabName;
  private NetworkTableEntry sb_wheel_calc;
  private NetworkTableEntry sb_yaw_calc;
  private NetworkTableEntry sb_trans;
  private NetworkTableEntry sb_yaw_set;
  private NetworkTableEntry sb_wheel_rpm_set;
  private NetworkTableEntry sb_apply;
  private NetworkTableEntry sb_control;

  // Block periodic until we're initialized
  private boolean           bInitialized = false;

  


  public AbstractSwerveModule(String name, String shuffleboardTabName)
  {
    // Save arguments
    this.name = name;
    this.shuffleboardTabName = shuffleboardTabName;

    // Initialize all of the shuffleboard inputs and outputs
    initShuffleboard();
  }

  /**
   * Initialize this module with the details provided by the robot-specific
   * subclass.
   *
   * @param gearRatioYaw
   *   The gear ratio, possibly only using a subset of the gear pairs in the
   *   module, which is responsible for the amount of yaw as a function of the
   *   two motors' speeds
   *
   * @param gearRatioWheelSpeed
   *   The gear ratio, typically consisting of all gear pairs in the module,
   *   responsible for the amount of wheel rotation as a function of the two
   *   motors' speed
   *
   * @param maxYawSpeedRpm
   *   The maximum speed that yaw should be allowed. This value is scaled by
   *   the encoder output, to adjust the yaw of the module
   *
   * @param motorA
   *   Instance of the first of the two motors used for differential swerve
   *
   * @param motorB
   *   Instance of the second of the two motors used for differential swerve
   *
   * @param encoder
   *   Instance of the encoder used for ascertaining the module's yaw
   *
   * @param wheelDiameterMeters
   *   The diameter of the module's wheel, in meters
   */
  public void initialize(double gearRatioYaw, double gearRatioWheelSpeed, double maxYawSpeedRpm,
                         IMotor motorA, IMotor motorB, IYawEncoder encoder, double wheelDiameterMeters)
  {
    // Save arguments
    this.motorA               = motorA;
    this.motorB               = motorB;
    yawEncoder                = encoder;
    GEAR_RATIO_YAW            = gearRatioYaw;
    GEAR_RATIO_WHEEL_SPEED    = gearRatioWheelSpeed;
    MAX_YAW_SPEED_RPM         = maxYawSpeedRpm;
    WHEEL_DIAMETER_METERS     = wheelDiameterMeters;

    // If we're using a relative encoder, assume the robot starts up
    // facing to the zero position. If we're using an absolute
    // encoder, the `setZero` method is a no-op.
    yawEncoder.setZero();

    // We're now initialized, so periodic() can be entered
    bInitialized = true;
  }

  /**
   * Reset module's encoder to angle zero
   */
  public void setZero()
  {
    yawEncoder.setZero();
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
    // Optimize to avoid large module rotations
    state = SwerveModuleState.optimize(state, this.getModuleState().angle);

    // Convert the state to our internally-used units, and save
    desiredWheelSpeedRPM = frc.bionic.Conversion.mpsToRpm(state.speedMetersPerSecond, WHEEL_DIAMETER_METERS);
    desiredYawDegrees = state.angle.getDegrees();

    // Keep shuffleboard up to date with provided state
    sb_yaw_set.setDouble(desiredYawDegrees);
    sb_wheel_rpm_set.setDouble(desiredWheelSpeedRPM);
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

    // If we're not yet initialized, wait for next time
    if (! bInitialized)
    {
      return;
    }

    // Determine the percentage of output, based on difference between
    // yaw goal and actual angle, to be used in the RPM calculation.
    calculatedYawRPM = yawEncoder.getOutputSignedPercent(desiredYawDegrees) * MAX_YAW_SPEED_RPM;

    if (!sb_control.getBoolean(true))
    {
      // Given the desired wheel and yaw RPM, calculate and specify the
      // motor speeds necessary to achieve them
      setMotorSpeedsRPM(desiredWheelSpeedRPM, calculatedYawRPM);
    }

    // report to dashboard
    sb_wheel_calc.setDouble(desiredWheelSpeedRPM);
    sb_yaw_calc.setDouble(calculatedYawRPM);

    // Ensure that our motor and encoder periodic functions are called, too
    motorA.periodic();
    motorB.periodic();
    yawEncoder.periodic();

    syncShuffleboard();
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
    // TODO: Is this calculation correct regardless of motor
    // orientation? Always subtract?

    // Translation is calculated as half the difference of a and b,
    // adjusted by gear ratio. Translation is typically dependent on
    // all three pairs of gears. The differential pinion generates
    // translation as a function of the speed of the top and bottom
    // differential gears.
    return ((aMotorRPM - bMotorRPM) / 2) * GEAR_RATIO_WHEEL_SPEED ;
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
    // TODO: Is this calculation correct regardless of motor
    // orientation? Always add?

    // Yaw is calculated as the average of a and b, adjusted by gear ratio
    return ((aMotorRPM + bMotorRPM) / 2) * GEAR_RATIO_YAW;
  }

  /**
   * Calculate the motor A RPM. Calculating motor A and B RPMs require
   * combinding the wheel and yaw RPMs, but the sign of the
   * combination operation varies depending on module design. We
   * therefore let the extending class implement each of these.
   *
   * The implementation will like look like this:
   *     return (wheelRPM / GEAR_RATIO_WHEEL_SPEED) + (yawRPM / GEAR_RATIO_YAW);
   * or this:
   *     return (wheelRPM / GEAR_RATIO_WHEEL_SPEED) - (yawRPM / GEAR_RATIO_YAW);
   */
  abstract protected double getARPM(double wheelRPM, double yawRPM);

  /**
   * Calculate the motor B RPM. Calculating motor A and B RPMs require
   * combinding the wheel and yaw RPMs, but the sign of the
   * combination operation varies depending on module design. We
   * therefore let the extending class implement each of these.
   *
   * The implementation will like look like this:
   *     return (wheelRPM / GEAR_RATIO_WHEEL_SPEED) + (yawRPM / GEAR_RATIO_YAW);
   * or this:
   *     return (wheelRPM / GEAR_RATIO_WHEEL_SPEED) - (yawRPM / GEAR_RATIO_YAW);
   */
  abstract protected double getBRPM(double wheelRPM, double yawRPM);

  /**
   * Convert desired translation and yaw RPMs to motor RPMs
   */
  protected void setMotorSpeedsRPM(double wheelRPM, double yawRPM)
  {
    double aRPM =  getARPM(wheelRPM, yawRPM);
    double bRPM =  getBRPM(wheelRPM, yawRPM);

    motorA.setGoalRPM(aRPM);
    motorB.setGoalRPM(bRPM);
  }

  /**
   * Initialize the shuffleboard interface for this motor
   */
  protected void initShuffleboard()
  {
    ShuffleboardTab           tab;
    ShuffleboardLayout        layout;

    tab              = Shuffleboard.getTab(shuffleboardTabName);
    layout           = tab.getLayout("Module " + name, BuiltInLayouts.kList);

    sb_trans         = layout.add("translation",  0).getEntry();

    sb_wheel_calc    = layout.add("calc wheel rpm", 0).getEntry();
    sb_yaw_calc      = layout.add("calc yaw rpm", 0).getEntry();

    sb_wheel_rpm_set = layout.add("wheel set rpm",  0).getEntry();
    sb_yaw_set       = layout.add("yaw set deg",  0).getEntry();
    sb_apply         = layout.add("Apply", false).getEntry();

    // TODO set back to false by default
    sb_control       = layout.add("Control", true).getEntry();
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

    // allow overriding settings during running test
    if (sb_apply.getBoolean(false))
    {
      // reset pushbutton so it's ready for additional user changes
      sb_apply.setBoolean(false);

      // Save desired settings
      desiredWheelSpeedRPM = sb_wheel_rpm_set.getDouble(0);
      desiredYawDegrees = sb_yaw_set.getDouble(0);

      // periodic() will use the above values on its next invocation
    }

    // get the motor speeds
    aMotorRPM = motorA.getVelocityRPM();
    bMotorRPM = motorB.getVelocityRPM();

    // calculate the wheel speed from those motor speeds
    currentWheelRPM = motorRPMsToWheelRPM(aMotorRPM, bMotorRPM);

    // report to shuffleboard
    sb_trans.setDouble(currentWheelRPM);
  }
}
