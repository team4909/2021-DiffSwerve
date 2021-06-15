package frc.robot.subsystems.shooter;

import javax.swing.text.Position;

import com.revrobotics.CANEncoder;
import com.revrobotics.CANError;
import com.revrobotics.CANPIDController;
import com.revrobotics.CANSparkMax;
import com.revrobotics.ControlType;
import com.revrobotics.EncoderType;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.Spark;
import edu.wpi.first.wpilibj.controller.PIDController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj.trajectory.TrapezoidProfile.Constraints;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.bionic.SlewRateLimiter;

public class HoodSubsystem extends SubsystemBase{


    CANSparkMax hoodMotor;
    CANPIDController hoodPID;
    CANEncoder hoodEncoder;
    double hoodPos;
    SlewRateLimiter slewRateLimiter;

    private final TrapezoidProfile.Constraints constraints =
        new TrapezoidProfile.Constraints(1.75, 0.75);

    private TrapezoidProfile.State goal = new TrapezoidProfile.State();
    private TrapezoidProfile.State setpoint = new TrapezoidProfile.State();

    //There needs to be a 'd' at the end of the constants to explicitly mark it as a double! 
    // How many ticks are in one reovlution of a motor
    int TICKS_PER_REVOLUTION_MOTOR = 42;
    // How many rotations of a motor turns the gear
    double REVOLUTIONS_MOTOR_PER_REVOLUTION_OUTPUT_SHAFT = 16d;
    // How much the hood moves per rotation of the gear
    double REVOLUTION_HOOD_PER_REVOLUTION_GEAR = 400d/69d;
    // How much the hood moves per degree
    double REVOLUTION_HOOD_PER_DEGREES = 1d/360d;

    public HoodSubsystem(){
        hoodMotor = new CANSparkMax(12, MotorType.kBrushless);
        hoodMotor.restoreFactoryDefaults();

        hoodPID = hoodMotor.getPIDController();

        // TODO: Encoder takes in Channel A, Channel B, and reverse Direction
        hoodEncoder = hoodMotor.getEncoder(EncoderType.kHallSensor, TICKS_PER_REVOLUTION_MOTOR);
        // hoodMotor.setClosedLoopRampRate(1);
        // hoodMotor.setOpenLoopRampRate(1);
        hoodMotor.setSmartCurrentLimit(1);
        // Turns the position of the hood, so one "tick" of the motor is equal to one degree of the hood
        // hoodEncoder.setPositionConversionFactor(REVOLUTIONS_MOTOR_PER_REVOLUTION_OUTPUT_SHAFT * 
        //                                          REVOLUTION_HOOD_PER_REVOLUTION_GEAR *
        //                                          REVOLUTION_HOOD_PER_DEGREES);

        hoodEncoder.setPosition(0);


    }

    public void periodic(){
        SmartDashboard.putNumber("Encoder Position", hoodEncoder.getPosition());
        SmartDashboard.putNumber("Hood Position", hoodPos);
        // SmartDashboard.putNumber("PID CALCULATE", pidCalc);

        TrapezoidProfile profile = new TrapezoidProfile(constraints, goal, setpoint);
        setpoint = profile.calculate(0.02);
        hoodPID.setReference(setpoint.position, ControlType.kPosition);

    }

    public double getHoodPosition() {
        return hoodEncoder.getPosition();
    }

    public void moveHoodUp(){

        if (hoodPos < 5){ //8.96
            hoodPos += 1;
            goal = new TrapezoidProfile.State(hoodPos, 0);
        }
    }

    public void moveHoodDown(){
        if (hoodPos > 0){
            hoodPos -= 1;
            goal = new TrapezoidProfile.State(hoodPos, 0);
        }
    }
 

    public void zeroHood(){
        hoodPos = 0;
    }
    
}
