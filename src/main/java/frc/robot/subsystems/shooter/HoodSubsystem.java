package frc.robot.subsystems.shooter;

import javax.swing.text.Position;

import com.revrobotics.CANEncoder;
import com.revrobotics.CANPIDController;
import com.revrobotics.CANSparkMax;
import com.revrobotics.EncoderType;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.Spark;
import edu.wpi.first.wpilibj.controller.PIDController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class HoodSubsystem extends SubsystemBase{


    CANSparkMax hoodMotor;
    PIDController hoodPID;
    CANEncoder hoodEncoder;
    double hoodPos;

    // How many ticks are in one reovlution of a motor
    double TICKS_PER_REVOLUTION_MOTOR = 42;
    // How many rotations of a motor turns the gear
    double REVOLUTIONS_MOTOR_PER_REVOLUTION_OUTPUT_SHAFT = 16;
    // How much the hood moves per rotation of the gear
    double REVOLUTION_HOOD_PER_REVOLUTION_GEAR = 400/69;
    // How much the hood moves per degree
    double REVOLUTION_HOOD_PER_DEGREES = 1/360;

    public HoodSubsystem(){
        hoodMotor = new CANSparkMax(12, MotorType.kBrushless);
        hoodMotor.restoreFactoryDefaults();

        hoodPID = new PIDController(0.1, 0, 0);

        // TODO: Encoder takes in Channel A, Channel B, and reverse Direction
        hoodEncoder = hoodMotor.getEncoder(EncoderType.kHallSensor, 42);
        // Turns the position of the hood, so one "tick" of the motor is equal to one degree of the hood
        hoodEncoder.setPositionConversionFactor(REVOLUTIONS_MOTOR_PER_REVOLUTION_OUTPUT_SHAFT * 
                                                REVOLUTIONS_MOTOR_PER_REVOLUTION_OUTPUT_SHAFT *
                                                REVOLUTION_HOOD_PER_REVOLUTION_GEAR *
                                                REVOLUTION_HOOD_PER_DEGREES);

        hoodEncoder.setPosition(0);

    }

    public void periodic(){
        double pidCalc = hoodPID.calculate(hoodPos, hoodEncoder.getPosition());
        SmartDashboard.putNumber("Encoder Position", hoodEncoder.getPosition());
        SmartDashboard.putNumber("Hood Position", hoodPos);
        SmartDashboard.putNumber("PID CALCULATE", pidCalc);

        hoodMotor.setVoltage(-pidCalc);

    }

    public double getHoodPosition() {
        return hoodEncoder.getPosition();
    }

    public void moveHoodUp(){
        if (hoodPos < 20){
            hoodPos += 0.5;
        }
    }

    public void moveHoodDown(){
        if (hoodPos > 0){
            hoodPos -= 1;
        }
    }
 

    public void zeroHood(){
        hoodPID.setSetpoint(0);
        hoodPos = 0;
    }
    
}
