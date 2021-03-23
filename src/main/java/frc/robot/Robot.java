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
import edu.wpi.first.wpilibj.geometry.Rotation2d;
import edu.wpi.first.wpilibj.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInLayouts;
import edu.wpi.first.wpilibj.shuffleboard.ComplexWidget;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardLayout;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj.shuffleboard.SimpleWidget;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Angle.AngleType;


public class Robot extends TimedRobot {
  
  // will need https://www.revrobotics.com/content/sw/max/sdk/REVRobotics.json
  private CANSparkMax m_driveMotorA;
  private CANSparkMax m_driveMotorB;
  private CANPIDController m_aPID;
  private CANPIDController m_bPID;

  private CANEncoder m_driveEncoderA;
  private CANEncoder m_driveEncoderB;
  private Encoder m_turningEncoder;

  private final int driveMotorChannelA = 1;
  private final int driveMotorChannelB = 2;

  private final int dioEncoderChanA = 0;
  private final int dioEncoderChanB = 1;

  // these are vectors of length 1 (unit vectors) used in the "power space"
  static final Vec2d MOTOR_1_VECTOR = new Vec2d(1/Math.sqrt(2), 1/Math.sqrt(2));
  static final Vec2d MOTOR_2_VECTOR = new Vec2d(-1/Math.sqrt(2), 1/Math.sqrt(2));

  private PIDController pid_drive;

  private ShuffleboardTab sb_tab;

  private ShuffleboardLayout sb_turnEnc;
  private NetworkTableEntry sb_turnDist;
  private NetworkTableEntry sb_turnVelo;

  private ShuffleboardLayout sb_inputs;
  private NetworkTableEntry sb_velocity, sb_heading, sb_a_volts, sb_b_volts, sb_drive_ff;

  private ShuffleboardLayout sb_motorA;
  private NetworkTableEntry sb_velA;
  private NetworkTableEntry sb_velA_avg;
  private NetworkTableEntry sb_voltsA;

  private ShuffleboardLayout sb_motorB;
  private NetworkTableEntry sb_velB;
  private NetworkTableEntry sb_velB_avg;
  private NetworkTableEntry sb_voltsB;

  private ShuffleboardLayout sb_output;
  private NetworkTableEntry  sb_rot;
  private NetworkTableEntry  sb_trans;

  private ShuffleboardLayout sb_debug;
  private NetworkTableEntry  sb_use_volts;
  private NetworkTableEntry  sb_pid_apply;

  private ShuffleboardLayout sb_pid;
  private NetworkTableEntry  sb_turn_calc;
  private NetworkTableEntry  sb_drive_calc;

  private ShuffleboardLayout sb_motor_pid;
  private ShuffleboardLayout sb_apid;
  private NetworkTableEntry  sb_apid_kp, sb_apid_ki, sb_apid_kd, sb_apid_kiz, sb_apid_kff, sb_apid_max, sb_apid_min;

  private ShuffleboardLayout sb_bpid;
  private NetworkTableEntry  sb_bpid_kp, sb_bpid_ki, sb_bpid_kd, sb_bpid_kiz, sb_bpid_kff, sb_bpid_max, sb_bpid_min;

  private final double MAX_VOLTAGE=10;

  double akp, aki, akd, akiz, akff, bkp, bki, bkd, bkiz, bkff;

  MedianFilter m_a_avg = new MedianFilter(50);
  MedianFilter m_b_avg = new MedianFilter(50);

  final double GEAR_RATIO_12  =  (80.0/10.0) * (90.0/34.0);
  final double GEAR_RATIO_123 =  (80.0/10.0) * (90.0/34.0) * (21.0/82.0);

  public void robotInit() {
    m_driveMotorA = new CANSparkMax(driveMotorChannelA, MotorType.kBrushless);
    m_driveEncoderA = m_driveMotorA.getEncoder();
    m_driveMotorA.restoreFactoryDefaults();
    m_aPID = m_driveMotorA.getPIDController();
    
    m_driveMotorB = new CANSparkMax(driveMotorChannelB, MotorType.kBrushless);
    m_driveEncoderB = m_driveMotorB.getEncoder();
    m_driveMotorB.restoreFactoryDefaults();
    m_bPID = m_driveMotorB.getPIDController();

    m_turningEncoder = new Encoder(dioEncoderChanA, dioEncoderChanB, true, EncodingType.k4X);
    final double kRad2RPM = 2 * Math.PI;
    final double kEncoderTicksPerRev = 128.0 * kRad2RPM; //grayhill encoder
    m_turningEncoder.setDistancePerPulse(360.0 / kEncoderTicksPerRev);

    pid_drive = new PIDController(.06, 0, 0);

    sb_tab = Shuffleboard.getTab("Swerve");

    sb_turnEnc   = sb_tab.getLayout("TurnEnc", BuiltInLayouts.kList);
    sb_turnDist  = sb_turnEnc.add("distance", 0).getEntry();
    sb_turnVelo  = sb_turnEnc.add("velocity", 0).getEntry();

    sb_inputs    = sb_tab.getLayout("Inputs", BuiltInLayouts.kList);
    sb_heading   = sb_inputs.add("heading",  0).getEntry();
    sb_velocity  = sb_inputs.add("velocity", 0).getEntry();
    sb_a_volts   = sb_inputs.add("a volts", 0).getEntry();
    sb_b_volts   = sb_inputs.add("b volts", 0).getEntry();

    sb_drive_ff   = sb_inputs.add("drive ff", .000175).getEntry();
    // sb_drive_ff   = sb_inputs.add("drive ff", 0).getEntry();

    sb_motorA    = sb_tab.getLayout("MotorA", BuiltInLayouts.kList);
    sb_velA      = sb_motorA.add("velocity",  0).getEntry();
    sb_velA_avg  = sb_motorA.add("velocity avg",  0).getEntry();
    sb_voltsA    = sb_motorA.add("volts",  0).getEntry();

    sb_motorB    = sb_tab.getLayout("MotorB", BuiltInLayouts.kList);
    sb_velB      = sb_motorB.add("velocity",  0).getEntry();
    sb_velB_avg  = sb_motorB.add("velocity avg",  0).getEntry();
    sb_voltsB    = sb_motorB.add("volts",  0).getEntry();

    sb_output    = sb_tab.getLayout("Output", BuiltInLayouts.kList);
    sb_rot       = sb_output.add("Rotation",  0).getEntry();
    sb_trans     = sb_output.add("Translation",  0).getEntry();

    sb_debug     = sb_tab.getLayout("Debug", BuiltInLayouts.kList);
    sb_use_volts = sb_debug.add("UseVolts", false).getEntry();
    sb_pid_apply = sb_debug.add("apply", false).getEntry();
    // sb_pid_turn  = sb_pid.add("TurnPID",  false).getEntry();

    sb_pid         = sb_tab.getLayout("PID Calc", BuiltInLayouts.kList);
    sb_turn_calc   = sb_pid.add("Rotation",  0).getEntry();
    sb_drive_calc  = sb_pid.add("Translation",  0).getEntry();

    sb_motor_pid   = sb_tab.getLayout("Motor PID", BuiltInLayouts.kList);
    sb_apid        = sb_motor_pid.getLayout("A PID", BuiltInLayouts.kList);
    sb_apid_kp     = sb_apid.addPersistent("kP",  .000_06  ).getEntry();
    sb_apid_ki     = sb_apid.addPersistent("kI",  0        ).getEntry();
    sb_apid_kd     = sb_apid.addPersistent("kD",  0        ).getEntry();
    sb_apid_kiz    = sb_apid.addPersistent("kIz", 0        ).getEntry();
    sb_apid_kff    = sb_apid.addPersistent("kFF", .000_175 ).getEntry();
    sb_apid_max    = sb_apid.addPersistent("max", .7       ).getEntry();
    sb_apid_min    = sb_apid.addPersistent("min", -.7      ).getEntry();
    
    sb_bpid        = sb_motor_pid.getLayout("B PID", BuiltInLayouts.kList);
    sb_bpid_kp     = sb_bpid.addPersistent("kP",  .000_06   ).getEntry();
    sb_bpid_ki     = sb_bpid.addPersistent("kI",  0         ).getEntry();
    sb_bpid_kd     = sb_bpid.addPersistent("kD",  0         ).getEntry();
    sb_bpid_kiz    = sb_bpid.addPersistent("kIz", 0         ).getEntry();
    sb_bpid_kff    = sb_bpid.addPersistent("kFF", .000_175  ).getEntry();
    sb_bpid_max    = sb_bpid.addPersistent("max", .7        ).getEntry();
    sb_bpid_min    = sb_bpid.addPersistent("min", -.7       ).getEntry();

    
    SmartDashboard.putBoolean("0", false);
    SmartDashboard.putBoolean("50", false);
    SmartDashboard.putBoolean("100", false);
    SmartDashboard.putBoolean("150", false);
    SmartDashboard.putBoolean("Apply", false);
    SmartDashboard.putNumber("Apply Value", 250);
  }

  public void robotPeriodic() {
    // outputs we always want updated
    sb_turnDist.setDouble(m_turningEncoder.getDistance());
    sb_turnVelo.setDouble(m_turningEncoder.getRate());
    
    sb_velA.setDouble(m_driveEncoderA.getVelocity());
    sb_velA_avg.setDouble(m_a_avg.calculate(m_driveEncoderA.getVelocity()));

    sb_velB.setDouble(m_driveEncoderB.getVelocity());
    sb_velB_avg.setDouble(m_b_avg.calculate(m_driveEncoderB.getVelocity()));


    akp = sb_apid_kp.getDouble(0);
    aki = sb_apid_ki.getDouble(0);
    akd = sb_apid_kd.getDouble(0);
    akiz = sb_apid_kiz.getDouble(0);
    akff = sb_apid_kff.getDouble(0);

    bkp = sb_bpid_kp.getDouble(0);
    bki = sb_bpid_ki.getDouble(0);
    bkd = sb_bpid_kd.getDouble(0);
    bkiz = sb_bpid_kiz.getDouble(0);
    bkff = sb_bpid_kff.getDouble(0);

    if (sb_pid_apply.getBoolean(false)) {
      sb_pid_apply.setBoolean(false);

      m_aPID.setP(akp);
      m_aPID.setI(aki);
      m_aPID.setD(akd);
      m_aPID.setIZone(akiz);
      m_aPID.setFF(akff);
      m_aPID.setOutputRange(sb_apid_min.getDouble(0), sb_apid_max.getDouble(0));

      m_bPID.setP(bkp);
      m_bPID.setI(bki);
      m_bPID.setD(bkd);
      m_bPID.setIZone(bkiz);
      m_bPID.setFF(bkff);
      m_bPID.setOutputRange(sb_bpid_min.getDouble(0), sb_bpid_max.getDouble(0));
    }

    




    // sb_pid_drive.

    // sb_pid

    // double aVolts  = sb_a_volts.getDouble(0);
    // double bVolts = sb_b_volts.getDouble(0);

    // var res = Robot.getState(aVolts, bVolts);
    // SmartDashboard.putNumber("angle", res.angle.getDegrees());
    // SmartDashboard.putNumber("speed", res.speedMetersPerSecond);
    
  }


  public void teleopPeriodic() {

    if (SmartDashboard.getBoolean("Apply", false)) {
      SmartDashboard.putBoolean("Apply", false);
      double val = SmartDashboard.getNumber("Apply Value", 0);
      sb_velocity.setDouble(val);
      sb_a_volts.setDouble(val);
      sb_b_volts.setDouble(-val);
    }
    if (SmartDashboard.getBoolean("0", false)) {
      SmartDashboard.putBoolean("0", false);
      sb_velocity.setDouble(0.0);
      sb_a_volts.setDouble(0.0);
      sb_b_volts.setDouble(0.0);
    }
    if (SmartDashboard.getBoolean("50", false)) {
      SmartDashboard.putBoolean("50", false);
      sb_velocity.setDouble(50.0);
      sb_a_volts.setDouble(50.0);
      sb_b_volts.setDouble(-50.0);
    }
    if (SmartDashboard.getBoolean("100", false)) {
      SmartDashboard.putBoolean("100", false);
      sb_velocity.setDouble(100.0);
      sb_a_volts.setDouble(100.0);
      sb_b_volts.setDouble(-100.0);
    }
    if (SmartDashboard.getBoolean("150", false)) {
      SmartDashboard.putBoolean("150", false);
      sb_velocity.setDouble(150.0);
      sb_a_volts.setDouble(150.0);
      sb_b_volts.setDouble(-150.0);
    }

    double a = m_driveEncoderA.getVelocity();
    double b = m_driveEncoderB.getVelocity();

    double actualDriveVelocity = getWheelVelocity(a, b);

    // report to dashboard
    sb_rot.setDouble(getModuleYaw(a, b));
    sb_trans.setDouble(actualDriveVelocity);

    // step 1 - tune the motor internal PIDs
    if (sb_use_volts.getBoolean(false)) {
      double aVolts = sb_a_volts.getDouble(0);
      double bVolts = sb_b_volts.getDouble(0);
      //-12 to 12 input
      m_aPID.setReference(aVolts, ControlType.kVelocity);
      m_bPID.setReference(bVolts, ControlType.kVelocity);

       // report to dashboard
      sb_voltsA.setDouble(-1);
      sb_voltsB.setDouble(-1);
      return;
    }

    

    

    

    
    
    double desiredTurnVelocity  = sb_heading.getDouble(0);  //degrees, 0=from home to down-field; degrees increase clockwise
    double desiredDriveVelocity = sb_velocity.getDouble(0); //input is [-1,1]

    double driveFF = sb_drive_ff.getDouble(0);
    if (desiredDriveVelocity == 0) {
      driveFF = 0;
    }

    // double calcTurnVelocity  = 
    double calcDriveVelocity = desiredDriveVelocity; //pid_drive.calculate(actualDriveVelocity, desiredDriveVelocity) + driveFF;

    sb_drive_calc.setDouble(calcDriveVelocity);

    var velocities = getMotorSpeeds(calcDriveVelocity, desiredTurnVelocity);

    // MotorRPMs res = Robot.calcMotorPowers(new Vec2d(calcDriveVelocity, desiredTurnVelocity), MAX_VOLTAGE);
    double aVel = velocities.a;
    double bVel = velocities.b;

    

    //-12 to 12 input
    // m_driveMotorA.set( aVel );
    // m_driveMotorB.set( bVel );
    m_aPID.setReference(aVel, ControlType.kVelocity);
    m_bPID.setReference(bVel, ControlType.kVelocity);

    // report to dashboard
    sb_voltsA.setDouble( -1 );
    sb_voltsB.setDouble( -1 );
  }


  // wheel translation RPM
  public double getWheelVelocity(double aMotorRPM, double bMotorRPM) {
    //final double GEAR_RATIO_123 =  (80.0/10.0) * (90.0/34.0) * (21.0/82.0);
    double wheelRPM = ((aMotorRPM - bMotorRPM) / (GEAR_RATIO_123 * 2) );
    return wheelRPM;
  }

  public double getModuleYaw(double aMotorRPM, double bMotorRPM) {
    // final double GEAR_RATIO_12  =  (80.0/10.0) * (90.0/34.0);
    double moduleYawRPM = ((aMotorRPM + bMotorRPM) / 2) / GEAR_RATIO_12;
    return moduleYawRPM;
  }

  public MotorRPMs getMotorSpeeds(double translateRPM, double rotateRPM) {
    // solve the equations in getWheelVelocity for a and b
    double a = (rotateRPM * GEAR_RATIO_12) + (translateRPM * GEAR_RATIO_123);
    double b = (rotateRPM * GEAR_RATIO_12) - (translateRPM * GEAR_RATIO_123);
    return new MotorRPMs(a, b);
  }

  public void teleopInit() { 
    m_turningEncoder.reset(); //reset the encoder on teleop init (for testing)
  }

  public void autonomousInit() {}
  public void autonomousPeriodic() {}
  public void disabledInit() {}
  public void disabledPeriodic() {}
  public void testInit() {
    teleopInit();

    sb_apid_kp.setDouble(akp);
    sb_apid_ki.setDouble(aki);
    sb_apid_kd.setDouble(akd);
    sb_apid_kiz.setDouble(akiz);
    sb_apid_kff.setDouble(akff);

    sb_bpid_kp.setDouble(bkp);
    sb_bpid_ki.setDouble(bki);
    sb_bpid_kd.setDouble(bkd);
    sb_bpid_kiz.setDouble(bkiz);
    sb_bpid_kff.setDouble(bkff);
  }
  public void testPeriodic() {
    teleopPeriodic();
  }
}
