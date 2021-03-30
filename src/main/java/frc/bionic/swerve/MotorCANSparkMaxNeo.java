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

import com.revrobotics.CANEncoder;
import com.revrobotics.CANPIDController;
import com.revrobotics.CANSparkMax;
import com.revrobotics.ControlType;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import edu.wpi.first.wpilibj.MedianFilter;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInLayouts;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInWidgets;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardLayout;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;


public class MotorCANSparkMaxNeo implements IMotor
{
  // Initial, default PID constants, overridden by persistent shuffleboard fields
  private static final double kMotorP  = 0.000090;
  private static final double kMotorI  = 0.000001;
  private static final double kMotorD  = 0.000090;
  private static final double kMotorIz = 100.0;
  private static final double kMotorFf = 0.000090;

  // To keep the motor closer to peek power, we limit the max output.
  // See torque/speed curves on https://motors.vex.com/
  private static final double kMotorMax = .7;
  private static final double kMotorMin = -.7;

  // Devices, Sensors, Actuators
  private CANSparkMax       motorController;  // the motor controller
  private CANEncoder        encoder;          // internal encoder
  private CANPIDController  pid;              // internal PID controller
  private MedianFilter      velAverage;       // for displaying average RPM

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
   * Constructor for a CAN Spark Max / Neo motor implementation
   *
   * @param canDeviceId
   *   The PWM channel to which this motor controller is connected
   *
   * @param name
   *   Name used for debugging and for shuffleboard fields
   *
   * @param shuffleboardTabName
   *   Tab on which to place shuffleboard fields
   */
  public MotorCANSparkMaxNeo(int canDeviceId, String name, String shuffleboardTabName)
  {
    // Save parameter values used elsewhere
    this.name = name;
    this.shuffleboardTabName = shuffleboardTabName;

    // Get access to the specified motor controller
    motorController = new CANSparkMax(canDeviceId, MotorType.kBrushless);
    motorController.restoreFactoryDefaults();

    // Get the encoder and PID controller from the motor controller
    encoder = motorController.getEncoder();
    pid = motorController.getPIDController();

    // @todo current limits on the motor

    // Set zero RPM (motor stopped), initially
    setGoalRPM(0.0);

    // Prepare to display (on shuffleboard) recent average RPM
    velAverage = new MedianFilter(50);

    pid.setP(kMotorP);
    pid.setI(kMotorI);
    pid.setD(kMotorD);
    pid.setIZone(kMotorIz);
    pid.setFF(kMotorFf);
    pid.setOutputRange(kMotorMin, kMotorMax);

    // Initialize the shuffleboard interface
    initShuffleboard();
  }

  // interface implementation
  public void setGoalRPM(double goalRPM)
  {
    SmartDashboard.putNumber(name + " goal", goalRPM);
    pid.setReference(goalRPM, ControlType.kVelocity);
  }

  // interface implementation
  public double getVelocityRPM()
  {
    return encoder.getVelocity();
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
    sb_pid_kp      = sublayout.addPersistent("kP", kMotorP).getEntry();
    sb_pid_ki      = sublayout.addPersistent("kI", kMotorI).getEntry();
    sb_pid_kd      = sublayout.addPersistent("kD", kMotorD).getEntry();
    sb_pid_kiz     = sublayout.addPersistent("kIz", kMotorIz).getEntry();
    sb_pid_kff     = sublayout.addPersistent("kFF", kMotorFf).getEntry();
    sb_pid_max     = sublayout.addPersistent("max", kMotorMax).getEntry();
    sb_pid_min     = sublayout.addPersistent("min", kMotorMin).getEntry();
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
    sb_vel_rpm.setDouble(encoder.getVelocity());
    sb_vel_avg.setDouble(velAverage.calculate(encoder.getVelocity()));


    //////////////////////////////////////////////////////////////////////
    // PID
    //////////////////////////////////////////////////////////////////////

    // allow overriding settings during running test
    if (sb_pid_apply.getBoolean(false))
    {
      // reset pushbutton so it's ready for additional user changes
      sb_pid_apply.setBoolean(false);

      // set PID constants based on shuffleboard input
      pid.setP(sb_pid_kp.getDouble(0));
      pid.setI(sb_pid_ki.getDouble(0));
      pid.setD(sb_pid_kd.getDouble(0));
      pid.setIZone(sb_pid_kiz.getDouble(0));
      pid.setFF(sb_pid_kff.getDouble(0));
      pid.setOutputRange(sb_pid_max.getDouble(0), sb_pid_min.getDouble(0));
    }
  }
}
