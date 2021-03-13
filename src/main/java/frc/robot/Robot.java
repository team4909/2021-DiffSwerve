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
import edu.wpi.first.wpilibj.shuffleboard.BuiltInLayouts;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardLayout;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj.shuffleboard.SimpleWidget;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;


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

  private ShuffleboardTab m_tab;

  private ShuffleboardLayout m_turnEnc;
  private NetworkTableEntry m_turnDist;
  private NetworkTableEntry m_turnVelo;

  private ShuffleboardLayout m_inputs;
  private NetworkTableEntry m_velocity;
  private NetworkTableEntry m_heading;

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
  }


  public void robotPeriodic() {
    m_turnDist.setDouble(m_turningEncoder.getDistance());
    m_turnVelo.setDouble(m_turningEncoder.getRate());
  }


  public void teleopPeriodic() {
    double heading  = m_heading.getDouble(0);
    double velocity = m_velocity.getDouble(0);


  }

  public void teleopInit() {
    m_turningEncoder.reset(); //reset the encoder on teleop init (for testing)
  }


  public void autonomousInit() {}


  public void autonomousPeriodic() {}


  


  public void disabledInit() {}


  public void disabledPeriodic() {}


  public void testInit() {}


  public void testPeriodic() {}
}
