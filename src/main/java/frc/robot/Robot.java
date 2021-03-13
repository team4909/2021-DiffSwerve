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

  static final Vec2d MOTOR_1_VECTOR = new Vec2d(1/Math.sqrt(2), 1/Math.sqrt(2));
  static final Vec2d MOTOR_2_VECTOR = new Vec2d(-1/Math.sqrt(2), 1/Math.sqrt(2));

  private ShuffleboardTab m_tab;

  private ShuffleboardLayout m_turnEnc;
  private NetworkTableEntry m_turnDist;
  private NetworkTableEntry m_turnVelo;

  private ShuffleboardLayout m_inputs;
  private NetworkTableEntry m_velocity;
  private NetworkTableEntry m_heading;
  private NetworkTableEntry m_a_volts;
  private NetworkTableEntry m_b_volts;

  private ShuffleboardLayout m_motorA;
  private NetworkTableEntry m_velA;
  private NetworkTableEntry m_motor_voltsA;

  private ShuffleboardLayout m_motorB;
  private NetworkTableEntry m_velB;
  private NetworkTableEntry m_motor_voltsB;

  public void robotInit() {
    m_driveMotorA = new CANSparkMax(driveMotorChannelA, MotorType.kBrushless);
    m_driveEncoderA = m_driveMotorA.getEncoder();
    
    m_driveMotorB = new CANSparkMax(driveMotorChannelB, MotorType.kBrushless);
    m_driveEncoderB = m_driveMotorB.getEncoder();

    m_turningEncoder = new Encoder(dioEncoderChanA, dioEncoderChanB);
    final int kEncoderResolution = 128; //grayhill encoder
    m_turningEncoder.setDistancePerPulse(360 / kEncoderResolution);

    m_tab = Shuffleboard.getTab("Swerve");

    m_turnEnc   = m_tab.getLayout("TurnEnc", BuiltInLayouts.kList);
    m_turnDist  = m_turnEnc.add("distance", 0).getEntry();
    m_turnVelo  = m_turnEnc.add("velocity", 0).getEntry();

    m_inputs    = m_tab.getLayout("Inputs", BuiltInLayouts.kList);
    m_heading   = m_inputs.add("heading",  0).getEntry();
    m_velocity  = m_inputs.add("velocity", 0).getEntry();
    m_a_volts   = m_inputs.add("a volts", 0).getEntry();
    m_b_volts   = m_inputs.add("b volts", 0).getEntry();

    m_motorA    = m_tab.getLayout("MotorA", BuiltInLayouts.kList);
    m_velA      = m_motorA.add("velocity",  0).getEntry();

    m_motorB    = m_tab.getLayout("MotorB", BuiltInLayouts.kList);
    m_velB      = m_motorB.add("velocity",  0).getEntry();
  }

  public void robotPeriodic() {
    // outputs we always want updated
    m_turnDist.setDouble(m_turningEncoder.getDistance());
    m_turnVelo.setDouble(m_turningEncoder.getRate());
    m_velA.setDouble(m_driveEncoderA.getVelocity());
    m_velB.setDouble(m_driveEncoderB.getVelocity());
  }


  public void teleopPeriodic() {
    double desiredHeading  = m_heading.getDouble(0);
    double desiredVelocity = m_velocity.getDouble(0);

    double a = m_driveEncoderA.getVelocity();
    double b = m_driveEncoderB.getVelocity();


    // first test to do is to see what the range is on voltage
    double aVolts  = m_a_volts.getDouble(0);
    double bVolts = m_b_volts.getDouble(0);

    m_driveMotorA.setVoltage(aVolts);
    m_driveMotorB.setVoltage(bVolts);

    m_motor_voltsA.setDouble(aVolts);
    m_motor_voltsB.setDouble(bVolts);
  }

  public void teleopInit() {
    m_turningEncoder.reset(); //reset the encoder on teleop init (for testing)
  }

  public static SwerveModuleState getState(double aVel, double bVel) {
    
    Vec2d vecA = MOTOR_1_VECTOR.scale(aVel);
    // System.out.println("A -- x:" + vecA.getX() + " y:" + vecA.getY() + " m:" + vecA.getMagnitude() + " a:" + vecA.getAngleDouble(AngleType.NEG_180_TO_180_CARTESIAN));

    Vec2d vecB = MOTOR_2_VECTOR.scale(bVel);
    // System.out.println("B -- x:" + vecB.getX() + " y:" + vecB.getY() + " m:" + vecB.getMagnitude() + " a:" + vecB.getAngleDouble(AngleType.NEG_180_TO_180_CARTESIAN));

    Vec2d powerVec = vecA.add(vecB);

    double speed = powerVec.getX();
    double turn = powerVec.getY();
    
    return new SwerveModuleState(speed, Rotation2d.fromDegrees(turn));
  }


  public void autonomousInit() {}
  public void autonomousPeriodic() {}
  public void disabledInit() {}
  public void disabledPeriodic() {}
  public void testInit() {}
  public void testPeriodic() {}
}
