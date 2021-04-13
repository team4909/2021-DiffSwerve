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

import java.util.Map;

import com.ctre.phoenix.motorcontrol.TalonFXControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonFX;

import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.wpilibj.MedianFilter;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInLayouts;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInWidgets;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardLayout;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

/**
 * MotorFalcon500
 */
public class MotorFalcon500 implements IMotor{
  // Initial, default PID constants, overridden by persistent shuffleboard fields
  private static final double kMotorP = 0.09;
  private static final double kMotorI = 0.0;
  private static final double kMotorD = 0.0;
  private static final double kMotorIz = 100.0;
  private static final double kMotorFf = 0.0475;
  private static final int kMotorSlot = 0;

  // To keep the motor closer to peek power, we limit the max output.
  // See torque/speed curves on https://motors.vex.com/
  private static final double kMotorMax = 0.7;
  private static final double kMotorMin = -0.7;

  // Devices, Sensors, Actuators
  private MedianFilter velAverage; //For displaying average RPM
  private TalonFX motor;

  // Shuffleboard-related
  private String            name;
  private String            shuffleboardTabName;

  NetworkTableEntry         sb_vel_rpm;
  NetworkTableEntry         sb_vel_avg;
  NetworkTableEntry         sb_vel_rpm_set;
  NetworkTableEntry         sb_vel_apply;

  NetworkTableEntry         sb_pid_kp;
  NetworkTableEntry         sb_pid_ki;
  NetworkTableEntry         sb_pid_kd;
  NetworkTableEntry         sb_pid_kiz;
  NetworkTableEntry         sb_pid_kff;
  NetworkTableEntry         sb_pid_max;
  NetworkTableEntry         sb_pid_min;
  NetworkTableEntry         sb_pid_apply;

  /**
   * Constructor for Falcon500/TalonFX
   *
   * @param deviceId
   *   The CAN channel to which this motor controller is connected
   *
   * @param name
   *   Name used for debugging and for shuffleboard fields
   *
   * @param shuffleboardTabName
   *   Tab on which to place shuffleboard fields
   */
  public MotorFalcon500(int deviceId, boolean bReverse, String name, String shuffleboardTabName){
    // Save parameter values used elsewhere
    this.name = name;
    this.shuffleboardTabName = shuffleboardTabName;

    // Makes a new Talon motor
    motor = new TalonFX(deviceId);

    // Resets the motor to default
    motor.configFactoryDefault();

    // Set zero RPM (motor stopped), initially
    setGoalRPM(0.0);

    // Prepare to display (on shuffleboard) recent average RPM
    velAverage = new MedianFilter(50);

    motor.config_kP(kMotorSlot, kMotorP);
    motor.config_kI(kMotorSlot, kMotorI);
    motor.config_kD(kMotorSlot, kMotorD);
    motor.config_IntegralZone(kMotorSlot, kMotorIz);
    motor.config_kF(kMotorSlot, kMotorFf);
//    motor.configPeakOutputForward(kMotorMax);
//    motor.configPeakOutputReverse(kMotorMin);

    // Initilize Shuffleboard Interface
    initShuffleboard();

  }

  // interface implementation
  public void setGoalRPM(double goalRPM)
  {
    SmartDashboard.putNumber(name + " goal", goalRPM);
    motor.set(TalonFXControlMode.Velocity, goalRPM);
  }

  // interface implementation
  public double getVelocityRPM()
  {
    return motor.getActiveTrajectoryVelocity(); //THIS IS PER 100MS, change if needed in RPM
  }

  // interface implementation
  public void periodic()
  {
    // Read values from shuffleboard; synchronize motor state to them
    syncShuffleboard();
  }

  /**
   * Initialize the shuffleboard interface for this motor
   */
  private void initShuffleboard()
  {
    ShuffleboardTab           tab;
    ShuffleboardLayout        layout;
    ShuffleboardLayout        sublayout;

    tab            = Shuffleboard.getTab(shuffleboardTabName);
    layout         = tab.getLayout("Motor " + name, BuiltInLayouts.kGrid);
    layout.withSize(3, 8);
    layout.withPosition(9, 3);
    layout.withProperties(Map.of("Number of columns", 2, "Number of rows", 1));

    sublayout      = layout.getLayout("Velocity", BuiltInLayouts.kGrid);
    sb_vel_rpm     = sublayout.add("rpm",  0).withWidget(BuiltInWidgets.kTextView).getEntry();
    sb_vel_avg     = sublayout.add("avg",  0).getEntry();
    sb_vel_rpm_set = sublayout.add("set rpm", 0).getEntry();
    sb_vel_apply   = sublayout.add("Apply", false).withWidget(BuiltInWidgets.kToggleButton).getEntry();

    sublayout      = layout.getLayout("PID", BuiltInLayouts.kList);
    sb_pid_kp      = sublayout.addPersistent("MotorFalcon500-kP", kMotorP).getEntry();
    sb_pid_ki      = sublayout.addPersistent("MotorFalcon500-kI", kMotorI).getEntry();
    sb_pid_kd      = sublayout.addPersistent("MotorFalcon500-kD", kMotorD).getEntry();
    sb_pid_kiz     = sublayout.addPersistent("MotorFalcon500-kIz", kMotorIz).getEntry();
    sb_pid_kff     = sublayout.addPersistent("MotorFalcon500-kFF", kMotorFf).getEntry();
    sb_pid_max     = sublayout.addPersistent("MotorFalcon500-max", kMotorMax).getEntry();
    sb_pid_min     = sublayout.addPersistent("MotorFalcon500-min", kMotorMin).getEntry();
    sb_pid_apply   = sublayout.add("Apply", false).getEntry();
  }

  /**
   * Update dynamic values on shuffleboard, and read values and reset based on
   * read values, any settable parameters.
   */
  private void syncShuffleboard()
  {
    //////////////////////////////////////////////////////////////////////
    // VELOCITY
    //////////////////////////////////////////////////////////////////////

    // allow overriding settings during running test
    if (sb_vel_apply.getBoolean(false))
    {
      // reset pushbutton so it's ready for additional user changes
      sb_vel_apply.setBoolean(false);

      // Set velocity based on shuffleboard input
      setGoalRPM(sb_vel_rpm_set.getDouble(0));
    }

    // display velocity and its recent average
    sb_vel_rpm.setDouble(getVelocityRPM());
    sb_vel_avg.setDouble(velAverage.calculate(getVelocityRPM()));


    //////////////////////////////////////////////////////////////////////
    // PID
    //////////////////////////////////////////////////////////////////////

    // allow overriding settings during running test
    if (sb_pid_apply.getBoolean(false))
    {
      // reset pushbutton so it's ready for additional user changes
      sb_pid_apply.setBoolean(false);

      // set PID constants based on shuffleboard input
      motor.config_kP(kMotorSlot, sb_pid_kp.getDouble(0));
      motor.config_kI(kMotorSlot, sb_pid_ki.getDouble(0));
      motor.config_kD(kMotorSlot, sb_pid_kd.getDouble(0));
      motor.config_IntegralZone(kMotorSlot, sb_pid_kiz.getDouble(0));
      motor.config_kF(kMotorSlot, sb_pid_kff.getDouble(0));
      motor.configPeakOutputForward(sb_pid_max.getDouble(0));
      motor.configPeakOutputReverse(sb_pid_max.getDouble(0));
    }
  }
}
