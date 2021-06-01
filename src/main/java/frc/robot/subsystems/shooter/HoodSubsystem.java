package frc.robot.subsystems.shooter;

import javax.swing.text.Position;

import com.revrobotics.CANEncoder;
import com.revrobotics.CANPIDController;
import com.revrobotics.CANSparkMax;
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
    Encoder hoodEncoder;
    double hoodPos;

    public HoodSubsystem(){
        hoodMotor = new CANSparkMax(2, MotorType.kBrushless);
        hoodMotor.restoreFactoryDefaults();

        hoodPID = new PIDController(2, 0, 0);

        // TODO: Encoder takes in Channel A, Channel B, and reverse Direction
        hoodEncoder = new Encoder(0,0,1);

    }

    public void periodic(){
        SmartDashboard.putNumber("Encoder Position", hoodEncoder.getDistance());
        SmartDashboard.putNumber("Hood Position", hoodPos);


    }

    public double getHoodPosition() {
        return hoodEncoder.getDistance();
    }

    public void setHoodAngle(double position){
       hoodPID.setSetpoint(position);
       hoodPos = position;


    }

    public void zeroHood(){
        hoodPID.setSetpoint(0);
        hoodPos = 0;
    }
    
}
