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

    double NUMBER_OF_TEETH = 69;
    double GEAR_RATIO = (1);

    public HoodSubsystem(){
        hoodMotor = new CANSparkMax(12, MotorType.kBrushless);
        hoodMotor.restoreFactoryDefaults();

        hoodPID = new PIDController(0.1, 0, 0);

        // TODO: Encoder takes in Channel A, Channel B, and reverse Direction
        hoodEncoder = hoodMotor.getEncoder(EncoderType.kHallSensor, (int)(NUMBER_OF_TEETH * GEAR_RATIO) / 360);
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
        if (hoodPos < 360){
            hoodPos += 1;
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
