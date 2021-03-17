// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.revrobotics.CANEncoder;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.geometry.Rotation2d;
import edu.wpi.first.wpilibj.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInLayouts;
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

  private ShuffleboardTab sb_tab;

  private ShuffleboardLayout sb_turnEnc;
  private NetworkTableEntry sb_turnDist;
  private NetworkTableEntry sb_turnVelo;

  private ShuffleboardLayout sb_inputs;
  private NetworkTableEntry sb_velocity;
  private NetworkTableEntry sb_heading;
  private NetworkTableEntry sb_a_volts;
  private NetworkTableEntry sb_b_volts;

  private ShuffleboardLayout sb_motorA;
  private NetworkTableEntry sb_velA;
  private NetworkTableEntry sb_voltsA;

  private ShuffleboardLayout sb_motorB;
  private NetworkTableEntry sb_velB;
  private NetworkTableEntry sb_voltsB;

  private ShuffleboardLayout sb_output;
  private NetworkTableEntry  sb_rot;
  private NetworkTableEntry  sb_trans;

  private final double MAX_VOLTAGE=10;

  public void robotInit() {
    m_driveMotorA = new CANSparkMax(driveMotorChannelA, MotorType.kBrushless);
    m_driveEncoderA = m_driveMotorA.getEncoder();
    
    m_driveMotorB = new CANSparkMax(driveMotorChannelB, MotorType.kBrushless);
    m_driveEncoderB = m_driveMotorB.getEncoder();

    m_turningEncoder = new Encoder(dioEncoderChanA, dioEncoderChanB);
    final int kEncoderResolution = 128; //grayhill encoder
    m_turningEncoder.setDistancePerPulse(-360 / kEncoderResolution);

    sb_tab = Shuffleboard.getTab("Swerve");

    sb_turnEnc   = sb_tab.getLayout("TurnEnc", BuiltInLayouts.kList);
    sb_turnDist  = sb_turnEnc.add("distance", 0).getEntry();
    sb_turnVelo  = sb_turnEnc.add("velocity", 0).getEntry();

    sb_inputs    = sb_tab.getLayout("Inputs", BuiltInLayouts.kList);
    sb_heading   = sb_inputs.add("heading",  0).getEntry();
    sb_velocity  = sb_inputs.add("velocity", 0).getEntry();
    sb_a_volts   = sb_inputs.add("a volts", 0).getEntry();
    sb_b_volts   = sb_inputs.add("b volts", 0).getEntry();

    sb_motorA    = sb_tab.getLayout("MotorA", BuiltInLayouts.kList);
    sb_velA      = sb_motorA.add("velocity",  0).getEntry();
    sb_voltsA = sb_motorA.add("volts",  0).getEntry();

    sb_motorB    = sb_tab.getLayout("MotorB", BuiltInLayouts.kList);
    sb_velB      = sb_motorB.add("velocity",  0).getEntry();
    sb_voltsB    = sb_motorB.add("volts",  0).getEntry();

    sb_output    = sb_tab.getLayout("Output", BuiltInLayouts.kList);
    sb_rot       = sb_output.add("Rotation",  0).getEntry();
    sb_trans      = sb_output.add("Translation",  0).getEntry();
  }

  public void robotPeriodic() {
    // outputs we always want updated
    sb_turnDist.setDouble(m_turningEncoder.getDistance());
    sb_turnVelo.setDouble(m_turningEncoder.getRate());
    
    sb_velA.setDouble(m_driveEncoderA.getVelocity());
    sb_velB.setDouble(m_driveEncoderB.getVelocity());

    // double aVolts  = sb_a_volts.getDouble(0);
    // double bVolts = sb_b_volts.getDouble(0);

    // var res = Robot.getState(aVolts, bVolts);
    // SmartDashboard.putNumber("angle", res.angle.getDegrees());
    // SmartDashboard.putNumber("speed", res.speedMetersPerSecond);
    
  }


  public void teleopPeriodic() {
    double desiredTurnVelocity  = sb_heading.getDouble(0);  //degrees, 0=from home to down-field; degrees increase clockwise
    double desiredDriveVelocity = sb_velocity.getDouble(0); //input is [-1,1]

    double a = m_driveEncoderA.getVelocity();
    double b = m_driveEncoderB.getVelocity();

    //chris
    double rot = a*.101 + b*.101;
    double tra = a*.091 + b*-.093;


    // double rot = ((a+b)/2) / 5;
    // double GEAR_RATIO_12 =  ( (80.0/10.0) * (90.0/34.0) );
    // double tra = ((a-b) / GEAR_RATIO_12)*2;

    
    sb_rot.setDouble(rot);
    sb_trans.setDouble(tra);
    
    
    double aVolts  = sb_a_volts.getDouble(0);
    double bVolts = sb_b_volts.getDouble(0);


    // MotorPowers res = Robot.calcMotorPowers(new Vec2d(desiredDriveVelocity, desiredTurnVelocity), MAX_VOLTAGE);
    // double aVolts  = res.a;
    // double bVolts = res.b;

    //-12 to 12 input
    m_driveMotorA.setVoltage( aVolts );
    m_driveMotorB.setVoltage( bVolts );

    // report to dashboard
    sb_voltsA.setDouble( aVolts );
    sb_voltsB.setDouble( bVolts );
  }

  public void teleopInit() {
    m_turningEncoder.reset(); //reset the encoder on teleop init (for testing)
  }

  public static SwerveModuleState getState(double aVel, double bVel) {
    
    Vec2d vecA = MOTOR_1_VECTOR.scale(aVel);
    //System.out.println("A -- x:" + vecA.getX() + " y:" + vecA.getY() + " m:" + vecA.getMagnitude() + " a:" + vecA.getAngleDouble(AngleType.NEG_180_TO_180_CARTESIAN));

    Vec2d vecB = MOTOR_2_VECTOR.scale(bVel);
    //System.out.println("B -- x:" + vecB.getX() + " y:" + vecB.getY() + " m:" + vecB.getMagnitude() + " a:" + vecB.getAngleDouble(AngleType.NEG_180_TO_180_CARTESIAN));

    Vec2d powerVec = vecA.add(vecB);

    double speed = powerVec.getX();
    double turn = powerVec.getY();
    
    return new SwerveModuleState(speed, Rotation2d.fromDegrees(turn));
  }

  //takes vector in power vector coordinate system
  // ^(x component is relative translation power and y component is relative MODULE rotation power)
  //calculates motor powers that will result in the desired ratio of module translation and module rotation
  //sets motors to appropriate powers
  public static MotorPowers calcMotorPowers (Vec2d powerVector, double MAX_MOTOR_POWER) {
    
    // final double MAX_MOTOR_POWER = 10; //motor power in volts?
    //this is one way to convert desired ratio of module translation and module rotation to motor powers
    //vectors are not strictly necessary for this, but made it easier to visualize
    //more documentation on this visualization method coming soon
    Vec2d motor1Unscaled = powerVector.projection(MOTOR_1_VECTOR);
    Vec2d motor2Unscaled = powerVector.projection(MOTOR_2_VECTOR);

    //makes sure no vector magnitudes exceed the maximum motor power
    Vec2d[] motorPowersScaled = Vec2d.batchNormalize(MAX_MOTOR_POWER, motor1Unscaled, motor2Unscaled);
    double motor1power = motorPowersScaled[0].getMagnitude();
    double motor2power = motorPowersScaled[1].getMagnitude();

    //this is to add sign to magnitude, which returns an absolute value
    if (motorPowersScaled[0].getAngleDouble(Angle.AngleType.NEG_180_TO_180_CARTESIAN) != MOTOR_1_VECTOR.getAngleDouble(Angle.AngleType.NEG_180_TO_180_CARTESIAN)) {
        motor1power *= -1;
    }
    if (motorPowersScaled[1].getAngleDouble(Angle.AngleType.NEG_180_TO_180_CARTESIAN) != MOTOR_2_VECTOR.getAngleDouble(Angle.AngleType.NEG_180_TO_180_CARTESIAN)) {
        motor2power *= -1;
    }

    return new MotorPowers(motor1power, motor2power);
}


  public void autonomousInit() {}
  public void autonomousPeriodic() {}
  public void disabledInit() {}
  public void disabledPeriodic() {}
  public void testInit() {}
  public void testPeriodic() {}
}
