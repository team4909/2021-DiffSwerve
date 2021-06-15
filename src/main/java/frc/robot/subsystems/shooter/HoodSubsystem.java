package frc.robot.subsystems.shooter;

import com.revrobotics.CANEncoder;
import com.revrobotics.CANPIDController;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import com.revrobotics.ControlType;
import com.revrobotics.EncoderType;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class HoodSubsystem extends SubsystemBase{


    CANSparkMax hoodMotor;
    CANPIDController hoodPID;
    CANEncoder hoodEncoder;
    double hoodPos;

    // Trapezoidal Profile Constraints, MaxVal and MaxAcell
    private final TrapezoidProfile.Constraints constraints =
        new TrapezoidProfile.Constraints(1.75, 0.75);

    private TrapezoidProfile.State goal = new TrapezoidProfile.State();
    private TrapezoidProfile.State setpoint = new TrapezoidProfile.State();

    //There needs to be a 'd' at the end of the constants to explicitly mark it as a double
    // How many ticks are in one reovlution of a motor
    int TICKS_PER_REVOLUTION_MOTOR = 42;
    // How many rotations of a motor turns the gear
    double REVOLUTIONS_MOTOR_PER_REVOLUTION_OUTPUT_SHAFT = 16d;
    // How much the hood moves per rotation of the gear
    double REVOLUTION_GEAR_PER_REVOLUTION_HOOD = 400d/69d;
    // How much the hood moves per degree
    double REVOLUTION_HOOD_PER_DEGREES = 1d/360d;
    // Converts motor ticks to hood degrees
    double MOTOR_TICKS_TO_HOOD_DEGREES =  TICKS_PER_REVOLUTION_MOTOR *
                                          REVOLUTIONS_MOTOR_PER_REVOLUTION_OUTPUT_SHAFT *
                                          REVOLUTION_GEAR_PER_REVOLUTION_HOOD *
                                          REVOLUTION_HOOD_PER_DEGREES;   
    
    double HOOD_DEGREES_TO_MOTOR_TICKS =  REVOLUTION_HOOD_PER_DEGREES /
                                          REVOLUTION_GEAR_PER_REVOLUTION_HOOD /
                                          REVOLUTIONS_MOTOR_PER_REVOLUTION_OUTPUT_SHAFT /
                                          TICKS_PER_REVOLUTION_MOTOR;

    

    // Max amount of degrees the hood is allowed to move
    double MAX_DEGREES_HOOD = 35;
    // Min amount of degrees the hood is allowed to move
    double MIN_DEGREES_HOOD = 0;
    // How much the hood is allowed to move per call
    double HOOD_INCREMENT_DEGREES = 0.5;

    public HoodSubsystem(){
        hoodMotor = new CANSparkMax(12, MotorType.kBrushless);
        hoodMotor.restoreFactoryDefaults();

        hoodPID = hoodMotor.getPIDController();

        // TODO: Encoder takes in Channel A, Channel B, and reverse Direction
        hoodEncoder = hoodMotor.getEncoder(EncoderType.kHallSensor, TICKS_PER_REVOLUTION_MOTOR);

        {
        // Turns the position of the hood, so one "tick" of the motor is equal to one degree of the hood
        /**
         *  TODO I think we are using this method wrong 
         *  documentation says that default is: REVOLUTIONS PER VOLT
         *  https://www.revrobotics.com/content/sw/max/sw-docs/SPARK-MAX-Java-API-Offline.pdf
        //  * */ 
        // hoodEncoder.setPositionConversionFactor(REVOLUTIONS_MOTOR_PER_REVOLUTION_OUTPUT_SHAFT * 
        //                                          REVOLUTION_HOOD_PER_REVOLUTION_GEAR *
        //                                          REVOLUTION_HOOD_PER_DEGREES);
        }   

        // Sets the P to 0.1
        hoodPID.setP(0.1);
        // Sets the I to 0
        hoodPID.setI(0d);
        // Sets the D to 0
        hoodPID.setD(0d);
        // Sets the output range (the min ad max the PID is allowed to go) to -1.5, 1.5 V
        hoodPID.setOutputRange(-1.5, 1.5);
                                        
        // Sets the position to zero on construct
        hoodEncoder.setPosition(0);


    }

    public void periodic(){
        SmartDashboard.putNumber("Encoder Position", hoodEncoder.getPosition());
        SmartDashboard.putNumber("Hood Position", hoodPos);

        // Creates a Trapezoid profile with the given constraints, goal state
        TrapezoidProfile profile = new TrapezoidProfile(constraints, goal);
        // Samples the newly created profile 20ms in the future and saves it in a state
        setpoint = profile.calculate(0.02);

        // Sets the setpoint of the PID as the future setpoint
        hoodPID.setReference(setpoint.position * MOTOR_TICKS_TO_HOOD_DEGREES, ControlType.kPosition);

    }

    public double getHoodPosition() {
        return hoodEncoder.getPosition();
    }

    public void moveHoodUp(){
        // Checks if the current degrees is less than the max degrees
        if (hoodEncoder.getPosition() * HOOD_DEGREES_TO_MOTOR_TICKS < MAX_DEGREES_HOOD){
            goal = new TrapezoidProfile.State((hoodEncoder.getPosition() * HOOD_DEGREES_TO_MOTOR_TICKS) + HOOD_INCREMENT_DEGREES, 0);
        }
    }

    public void moveHoodDown(){
        // Checks if the current degrees is more than the min degrees
        if (hoodEncoder.getPosition() * HOOD_DEGREES_TO_MOTOR_TICKS > MIN_DEGREES_HOOD){
            goal = new TrapezoidProfile.State((hoodEncoder.getPosition() * HOOD_DEGREES_TO_MOTOR_TICKS) - HOOD_INCREMENT_DEGREES, 0);
        }
    }
 

    public void zeroHood(){
        hoodPos = 0;
    }   
}
