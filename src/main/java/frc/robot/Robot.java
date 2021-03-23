// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.revrobotics.CANEncoder;
import com.revrobotics.CANPIDController;
import com.revrobotics.CANSparkMax;
import com.revrobotics.ControlType;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.MedianFilter;
import edu.wpi.first.wpilibj.controller.PIDController;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.CounterBase.EncodingType;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInLayouts;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardLayout;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;


public class Robot extends TimedRobot {
  
  // will need https://www.revrobotics.com/content/sw/max/sdk/REVRobotics.json
  private CANSparkMax m_driveMotorA;
  private CANSparkMax m_driveMotorB;
  private CANPIDController m_aPID;
  private CANPIDController m_bPID;

  private CANEncoder m_driveEncoderA;
  private CANEncoder m_driveEncoderB;
  private Encoder m_yawEncoder;

  private final int driveMotorChannelA = 1;
  private final int driveMotorChannelB = 2;

  private final int dioEncoderChanA = 0;
  private final int dioEncoderChanB = 1;

  private PIDController pid_yaw;

  private ShuffleboardTab sb_tab;

  private ShuffleboardLayout sb_yawEnc;
  private NetworkTableEntry sb_yawDist, sb_yawVelo;

  private ShuffleboardLayout sb_inputs;
  private NetworkTableEntry sb_velocity, sb_heading, sb_a_motorRPM, sb_b_motorRPM;

  private ShuffleboardLayout sb_motorA;
  private NetworkTableEntry sb_velA, sb_velA_avg;

  private ShuffleboardLayout sb_motorB;
  private NetworkTableEntry sb_velB, sb_velB_avg;

  private ShuffleboardLayout sb_output;
  private NetworkTableEntry  sb_yaw, sb_trans;

  private ShuffleboardLayout sb_debug;
  private NetworkTableEntry  sb_use_motorRPM, sb_pid_apply;

  private ShuffleboardLayout sb_pid;
  private NetworkTableEntry  sb_yaw_calc, sb_drive_calc;

  private ShuffleboardLayout sb_motor_pid, sb_apid;
  private NetworkTableEntry  sb_apid_kp, sb_apid_ki, sb_apid_kd, sb_apid_kiz, sb_apid_kff, sb_apid_max, sb_apid_min;

  private ShuffleboardLayout sb_bpid;
  private NetworkTableEntry  sb_bpid_kp, sb_bpid_ki, sb_bpid_kd, sb_bpid_kiz, sb_bpid_kff, sb_bpid_max, sb_bpid_min;

  MedianFilter m_a_avg = new MedianFilter(50);
  MedianFilter m_b_avg = new MedianFilter(50);

  // Gear ratio of first two pairs of gears
  // Yaw does not depend on the third pair
  final double GEAR_RATIO_12  =  (10.0/80.0) * (34.0/90.0);

  // Gear ratio of all three pairs of gears
  // Wheel speed depends on all three
  final double GEAR_RATIO_123 =  (10.0/80.0) * (34.0/90.0) * (82.0/21.0);

  final double kMotorP = 0.000090;
  final double kMotorI = 0.000001;
  final double kMotorD = 0.000090;
  final double kMotorIz = 100.0;
  final double kMotorFf = 0.000090;

  // To keep the motor closer to peek power, we limit the max output.
  // See torque/speed curves on https://motors.vex.com/
  final double kMotorMax = .7;
  final double kMotorMin = -.7;
  // @todo current limits on the motor

  public void robotInit() {
    m_driveMotorA = new CANSparkMax(driveMotorChannelA, MotorType.kBrushless);
    m_driveEncoderA = m_driveMotorA.getEncoder();
    m_driveMotorA.restoreFactoryDefaults();
    m_aPID = m_driveMotorA.getPIDController();
    m_aPID.setP(kMotorP);
    m_aPID.setI(kMotorI);
    m_aPID.setD(kMotorD);
    m_aPID.setIZone(kMotorIz);
    m_aPID.setFF(kMotorFf);
    m_aPID.setOutputRange(kMotorMin, kMotorMax);

    
    m_driveMotorB = new CANSparkMax(driveMotorChannelB, MotorType.kBrushless);
    m_driveEncoderB = m_driveMotorB.getEncoder();
    m_driveMotorB.restoreFactoryDefaults();
    m_bPID = m_driveMotorB.getPIDController();
    m_bPID.setP(kMotorP);
    m_bPID.setI(kMotorI);
    m_bPID.setD(kMotorD);
    m_bPID.setIZone(kMotorIz);
    m_bPID.setFF(kMotorFf);
    m_bPID.setOutputRange(kMotorMin, kMotorMax);

    m_yawEncoder = new Encoder(dioEncoderChanA, dioEncoderChanB, true, EncodingType.k4X);
    // final double kRad2RPM = 2 * Math.PI;
    final double kEncoderTicksPerRev = 128.0; //grayhill encoder
    final double kEncoderDegPerPulse = 360 / kEncoderTicksPerRev;
    m_yawEncoder.setDistancePerPulse(kEncoderDegPerPulse);

    // Create a PID to enforce specified module direction
    // See pid_yaw.calculate for an explanation of how the PID value is used
    // We attempted to set the kp near use but found it broke the shuffleboard
    // pid calibarion tooling.
    pid_yaw = new PIDController(2, 0, 0);

    // When values outside this range are fed into the setpoint of the PID
    // the requested value is mapped onto this range.
    pid_yaw.enableContinuousInput(-180.0, 180.0);

    // Initialize all of the shuffleboard inputs and outputs
    initShuffleboard();
  }

  public void robotPeriodic() {
    // shuffleboard outputs we always want updated
    sb_yawDist.setDouble(m_yawEncoder.getDistance());
    sb_yawVelo.setDouble(m_yawEncoder.getRate());
    
    sb_velA.setDouble(m_driveEncoderA.getVelocity());
    sb_velA_avg.setDouble(m_a_avg.calculate(m_driveEncoderA.getVelocity()));

    sb_velB.setDouble(m_driveEncoderB.getVelocity());
    sb_velB_avg.setDouble(m_b_avg.calculate(m_driveEncoderB.getVelocity()));



    // allow overriding settings of PIDs during running test
    if (sb_pid_apply.getBoolean(false)) {
      sb_pid_apply.setBoolean(false);

      m_aPID.setP(sb_apid_kp.getDouble(0));
      m_aPID.setI(sb_apid_ki.getDouble(0));
      m_aPID.setD(sb_apid_kd.getDouble(0));
      m_aPID.setIZone(sb_apid_kiz.getDouble(0));
      m_aPID.setFF(sb_apid_kff.getDouble(0));
      m_aPID.setOutputRange(sb_apid_min.getDouble(0), sb_apid_max.getDouble(0));

      m_bPID.setP(sb_bpid_kp.getDouble(0));
      m_bPID.setI(sb_bpid_ki.getDouble(0));
      m_bPID.setD(sb_bpid_kd.getDouble(0));
      m_bPID.setIZone(sb_bpid_kiz.getDouble(0));
      m_bPID.setFF(sb_bpid_kff.getDouble(0));
      m_bPID.setOutputRange(sb_bpid_min.getDouble(0), sb_bpid_max.getDouble(0));
    }

    SmartDashboard.putData("Yaw PID", pid_yaw);  
  }


  public void teleopPeriodic() {
    // read shuffleboard inputs
    velSetpointsShuffleboard();
    headingSetpointsShuffleboard();

    // get the motor speeds
    double aMotorRPM = m_driveEncoderA.getVelocity();
    double bMotorRPM = m_driveEncoderB.getVelocity();

    // calculate the wheel speeds from those motor speeds
    double currentWheelRPM      = motorRPMsToWheelRPM(aMotorRPM, bMotorRPM);
    double currentModuleYawRPM  = motorRPMsToModuleYawRPM(aMotorRPM, bMotorRPM);

    // report to dashboard
    sb_yaw.setDouble(currentModuleYawRPM);
    sb_trans.setDouble(currentWheelRPM);

    // step 1 - tune the motor internal PIDs
    // allow overriding desired motor speeds during running test
    if (sb_use_motorRPM.getBoolean(false)) {
      double aDesiredMotorRPM = sb_a_motorRPM.getDouble(0);
      double bDesiredMotorRPM = sb_b_motorRPM.getDouble(0);
     
      m_aPID.setReference(aDesiredMotorRPM, ControlType.kVelocity);
      m_bPID.setReference(bDesiredMotorRPM, ControlType.kVelocity);

      return;
    }
    
    // Get desired yaw and wheel speed
    double desiredYawDeg = sb_heading.getDouble(0);
    double desiredWheelRPM = sb_velocity.getDouble(0);

    // PID is being used to convert the yaw error into the RPM to get there. For example,
    // if the encoder is currently at 90 degrees and desired is 180 degrees, the error
    // is 90 degrees initially. With a kP value of 2, this would yield an RPM of 180. As
    // the error decreases, the RPM decreases until eventually the error is zero and thus
    // the RPM is zero. See the constructor for the PID values.
    double pidCalculatedYawRPM = pid_yaw.calculate(m_yawEncoder.getDistance(), desiredYawDeg);

    // report to dashboard
    sb_drive_calc.setDouble(desiredWheelRPM);
    sb_yaw_calc.setDouble(pidCalculatedYawRPM);

    // Given the desired wheel and yaw RPM, calculate the motor speeds
    // necessary to achieve them
    var desiredMotorSpeedsRPM = getMotorSpeedsRPM(desiredWheelRPM, pidCalculatedYawRPM);

    // Set the reference speeds (setpoints) in the two PIDs
    m_aPID.setReference(desiredMotorSpeedsRPM.a, ControlType.kVelocity);
    m_bPID.setReference(desiredMotorSpeedsRPM.b, ControlType.kVelocity);
  }

  public void teleopInit() {
    m_yawEncoder.reset(); //reset the encoder on teleop init (for testing)
  }

  public void autonomousInit() {
  }

  public void autonomousPeriodic() {
  }

  public void disabledInit() {
  }

  public void disabledPeriodic() {
  }

  public void testInit() {
    teleopInit();
  }
  public void testPeriodic() {
    teleopPeriodic();
  }

  // get wheel translation in RPM
  public double motorRPMsToWheelRPM(double aMotorRPM, double bMotorRPM) {
    // Translation is calculated as half the difference of a and b,
    // adjusted by gear ratio. Translation is dependent on all three
    // pairs of gears. The differential pinion generates translation 
    // as a function of the speed of the top and bottom differential gears.
    double wheelRPM = ((aMotorRPM - bMotorRPM) / 2) * GEAR_RATIO_123 ;
    return wheelRPM;
  }

  // get module yaw in RPM
  public double motorRPMsToModuleYawRPM(double aMotorRPM, double bMotorRPM) {
    // Yaw is calculated as the average of a and b, adjusted by gear ratio
    // Yaw does not depend on the third pair of gears, thus GEAR_RATIO_12
    // only includes the first two gear reductions.
    double moduleYawRPM = ((aMotorRPM + bMotorRPM) / 2) * GEAR_RATIO_12;
    return moduleYawRPM;
  }

  // convert desired translation and yaw RPMs to motor RPMs
  public MotorRPMs getMotorSpeedsRPM(double wheelRPM, double yawRPM) {
    double a = (yawRPM / GEAR_RATIO_12) + (wheelRPM / GEAR_RATIO_123);
    double b = (yawRPM / GEAR_RATIO_12) - (wheelRPM / GEAR_RATIO_123);
    return new MotorRPMs(a, b);
  }

  // create/initialize all of the shuffleboard inputs and outputs
  private void initShuffleboard() {
    sb_tab = Shuffleboard.getTab("Swerve");

    sb_yawEnc       = sb_tab.getLayout("TurnEnc", BuiltInLayouts.kList);
    sb_yawDist      = sb_yawEnc.add("distance", 0).getEntry();
    sb_yawVelo      = sb_yawEnc.add("velocity", 0).getEntry();

    sb_inputs       = sb_tab.getLayout("Inputs", BuiltInLayouts.kList);
    sb_heading      = sb_inputs.add("heading",  0).getEntry();
    sb_velocity     = sb_inputs.add("velocity", 0).getEntry();
    sb_a_motorRPM   = sb_inputs.add("a motor RPM", 0).getEntry();
    sb_b_motorRPM   = sb_inputs.add("b motor RPM", 0).getEntry();

    sb_motorA       = sb_tab.getLayout("MotorA", BuiltInLayouts.kList);
    sb_velA         = sb_motorA.add("velocity",  0).getEntry();
    sb_velA_avg     = sb_motorA.add("velocity avg",  0).getEntry();

    sb_motorB       = sb_tab.getLayout("MotorB", BuiltInLayouts.kList);
    sb_velB         = sb_motorB.add("velocity",  0).getEntry();
    sb_velB_avg     = sb_motorB.add("velocity avg",  0).getEntry();

    sb_output       = sb_tab.getLayout("Output", BuiltInLayouts.kList);
    sb_yaw          = sb_output.add("Rotation",  0).getEntry();
    sb_trans        = sb_output.add("Translation",  0).getEntry();

    sb_debug        = sb_tab.getLayout("Debug", BuiltInLayouts.kList);
    sb_use_motorRPM = sb_debug.add("Use Motor RPM", false).getEntry();
    sb_pid_apply    = sb_debug.add("apply", false).getEntry();

    sb_pid          = sb_tab.getLayout("PID Calc", BuiltInLayouts.kList);
    sb_yaw_calc     = sb_pid.add("Rotation",  0).getEntry();
    sb_drive_calc   = sb_pid.add("Translation",  0).getEntry();

    sb_motor_pid    = sb_tab.getLayout("Motor PID", BuiltInLayouts.kList);
    sb_apid         = sb_motor_pid.getLayout("A PID", BuiltInLayouts.kList);
    sb_apid_kp      = sb_apid.addPersistent("kP", kMotorP).getEntry();
    sb_apid_ki      = sb_apid.addPersistent("kI", kMotorI).getEntry();
    sb_apid_kd      = sb_apid.addPersistent("kD", kMotorD).getEntry();
    sb_apid_kiz     = sb_apid.addPersistent("kIz", kMotorIz).getEntry();
    sb_apid_kff     = sb_apid.addPersistent("kFF", kMotorFf).getEntry();
    sb_apid_max     = sb_apid.addPersistent("max", kMotorMax).getEntry();
    sb_apid_min     = sb_apid.addPersistent("min", kMotorMin).getEntry();

    sb_bpid         = sb_motor_pid.getLayout("B PID", BuiltInLayouts.kList);
    sb_bpid_kp      = sb_bpid.addPersistent("kP", kMotorP).getEntry();
    sb_bpid_ki      = sb_bpid.addPersistent("kI", kMotorI).getEntry();
    sb_bpid_kd      = sb_bpid.addPersistent("kD", kMotorD).getEntry();
    sb_bpid_kiz     = sb_bpid.addPersistent("kIz", kMotorIz).getEntry();
    sb_bpid_kff     = sb_bpid.addPersistent("kFF", kMotorFf).getEntry();
    sb_bpid_max     = sb_bpid.addPersistent("max", kMotorMax).getEntry();
    sb_bpid_min     = sb_bpid.addPersistent("min", kMotorMin).getEntry();

    SmartDashboard.putBoolean("0", false);
    SmartDashboard.putBoolean("50", false);
    SmartDashboard.putBoolean("100", false);
    SmartDashboard.putBoolean("150", false);
    SmartDashboard.putBoolean("Apply", false);
    SmartDashboard.putNumber("Apply Value", 250);
  }

  // set up for wheel velocity quick-change from shuffleboard
  public void velSetpointsShuffleboard() {
    if (SmartDashboard.getBoolean("Apply", false)) {
      SmartDashboard.putBoolean("Apply", false);
      double val = SmartDashboard.getNumber("Apply Value", 0);
      sb_velocity.setDouble(val);
      sb_a_motorRPM.setDouble(val);
      sb_b_motorRPM.setDouble(-val);
    }
    if (SmartDashboard.getBoolean("0", false)) {
      SmartDashboard.putBoolean("0", false);
      sb_velocity.setDouble(0.0);
      sb_a_motorRPM.setDouble(0.0);
      sb_b_motorRPM.setDouble(0.0);
    }
    if (SmartDashboard.getBoolean("50", false)) {
      SmartDashboard.putBoolean("50", false);
      sb_velocity.setDouble(50.0);
      sb_a_motorRPM.setDouble(50.0);
      sb_b_motorRPM.setDouble(-50.0);
    }
    if (SmartDashboard.getBoolean("100", false)) {
      SmartDashboard.putBoolean("100", false);
      sb_velocity.setDouble(100.0);
      sb_a_motorRPM.setDouble(100.0);
      sb_b_motorRPM.setDouble(-100.0);
    }
    if (SmartDashboard.getBoolean("150", false)) {
      SmartDashboard.putBoolean("150", false);
      sb_velocity.setDouble(150.0);
      sb_a_motorRPM.setDouble(150.0);
      sb_b_motorRPM.setDouble(-150.0);
    }
  }

  // set up for heading quick-change from shuffleboard
  public void headingSetpointsShuffleboard() {
    if (SmartDashboard.getBoolean("head _ Apply", false)) {
      SmartDashboard.putBoolean("head _ Apply", false);
      double val = SmartDashboard.getNumber("head _ Apply Value", 0);
      sb_velocity.setDouble(val);
      sb_a_motorRPM.setDouble(val);
      sb_b_motorRPM.setDouble(-val);
    }
    if (SmartDashboard.getBoolean("head _ 0", false)) {
      SmartDashboard.putBoolean("head _ 0", false);
      sb_velocity.setDouble(0.0);

    }
    if (SmartDashboard.getBoolean("head _ 90", false)) {
      SmartDashboard.putBoolean("head _ 90", false);
      sb_velocity.setDouble(90.0);

    }
    if (SmartDashboard.getBoolean("head _ -90", false)) {
      SmartDashboard.putBoolean("head _ -90", false);
      sb_velocity.setDouble(-90);
    }
    if (SmartDashboard.getBoolean("head _ 180", false)) {
      SmartDashboard.putBoolean("head _ 180", false);
      sb_velocity.setDouble(180.0);
    }
  }
}
