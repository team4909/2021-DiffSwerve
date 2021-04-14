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

import edu.wpi.first.wpilibj.DutyCycleEncoder;
import edu.wpi.first.wpilibj.controller.PIDController;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInLayouts;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardLayout;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class YawEncoderRevHex implements IYawEncoder
{
  // Devices, Sensors, Actuators
  private DutyCycleEncoder  encoder;
  private PIDController     pid;

  // Offset to zero position, provided to constructor (possibly overridden by dashboard)
  private double encoderOffset = 0.0;

  // Shuffleboard-related
  private String            name;
  private String            shuffleboardTabName;

  private NetworkTableEntry sb_angle;
  private NetworkTableEntry sb_goal;
  private NetworkTableEntry sb_output;
  

  /**
   * Constructor for a Grayhill 63R128 encoder implementation
   *
   * @param dioChannel
   *   The first of two digital IO channels to which this encoder is connected
   *
   * @param name
   *   Name used for debugging and for shuffleboard fields
   *
   * @param shuffleboardTabName
   *   Tab on which to place shuffleboard fields
   */
  public YawEncoderRevHex(int dioChannel, double initialEncoderOffset, String name, String shuffleboardTabName)
  {
    // Save parameter values used elsewhere
    this.name = name;
    this.shuffleboardTabName = shuffleboardTabName;
    this.encoderOffset = initialEncoderOffset;

    // Get access to the encoder
    encoder = new DutyCycleEncoder(dioChannel);

    // We want encoder "distance" in terms degrees difference from the
    // 0 point. This model Grayhill encoder yields 128 ticks per
    // revolution (non-configurable). We can therefore calculate the
    // rotation per tick, in degrees.
    encoder.setDistancePerRotation(360.0);

    // Create a PID. 
    pid = new PIDController(0.88, 0, 0);
    pid.enableContinuousInput(-180.0, 180.0);

    // Initialize the shuffleboard interface
    initShuffleboard();
  }

  // interface implementation
  public void periodic()
  {
    // Read values from shuffleboard; synchronize motor state to them
    syncShuffleboard();
  }

  // interface implementation
  public double getDistanceDegrees()
  {
    return frc.bionic.Conversion.normalize(encoder.getDistance() + encoderOffset, -180, 180);
  }

  // interface implementation
  public void setGoalDegrees(double goalDegrees)
  {
    SmartDashboard.putNumber("req goal deg", goalDegrees);
    pid.setSetpoint(goalDegrees);
  }

  /**
   * private getter for the goal in degrees
   */
  private double getGoalDegrees()
  {
    return pid.getSetpoint();
  }

  // interface implementation
  public double getOutputSignedPercent()
  {
    // Convert from an output in degrees error, to a fractional output.
    // Since we have `setContinuousInput(-180.0, 180.0)`, the PID will
    // always yield a result in that range. Scale it for the documented, 
    // standardized output of range [-1.0, 1.0].
    double pidCalc = pid.calculate(getDistanceDegrees(), getGoalDegrees());
    SmartDashboard.putNumber("yaw dist deg", getDistanceDegrees());
    SmartDashboard.putNumber("yaw goal deg", getGoalDegrees());
    SmartDashboard.putNumber("yaw pidCalc", pidCalc);
    return pidCalc / 180.0;
  }

  /**
   * Initialize the shuffleboard interface for this motor
   */
  private void initShuffleboard()
  {
    ShuffleboardTab           tab;
    ShuffleboardLayout        layout;
    
    tab            = Shuffleboard.getTab(shuffleboardTabName);
    layout         = tab.getLayout("Yaw Encoder " + name, BuiltInLayouts.kList);

    sb_angle       = layout.add("angle deg",  0).getEntry();
    sb_goal        = layout.add("goal deg",  0).getEntry();
    sb_output      = layout.add("output +-%",  0).getEntry();
  }
  
  /**
   * Update dynamic values on shuffleboard, and read values and reset based on
   * read values, any settable parameters.
   */
  private void syncShuffleboard()
  {
    // Display encoder attributes
    sb_angle.setDouble(getDistanceDegrees());
    sb_goal.setDouble(getGoalDegrees());
    sb_output.setDouble(getOutputSignedPercent());
  }
}
