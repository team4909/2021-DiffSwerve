package frc.robot.subsystems.shooter;

import com.revrobotics.CANEncoder;
import com.revrobotics.CANPIDController;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import com.revrobotics.ControlType;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class HoodSubsystem extends SubsystemBase {

    CANSparkMax hoodMotor;
    CANPIDController hoodPID;
    CANEncoder hoodEncoder;
    double hoodPos;

    // There needs to be a 'd' at the end of the constants to explicitly mark it as
    // a double
    // How many ticks are in one reovlution of a motor
    int TICKS_PER_REVOLUTION_MOTOR = 42;
    // How many rotations of a motor turns the gear
    double REVOLUTIONS_MOTOR_PER_REVOLUTION_OUTPUT_SHAFT = 20d;
    // How much the hood moves per rotation of the gear
    double REVOLUTION_GEAR_PER_REVOLUTION_HOOD = 400d / 69d;
    // How much the hood moves per degree
    double REVOLUTION_HOOD_PER_DEGREES = 1d / 360d;
    // Converts motor ticks to hood degrees
    double MOTOR_TICKS_TO_HOOD_DEGREES = REVOLUTIONS_MOTOR_PER_REVOLUTION_OUTPUT_SHAFT
            * REVOLUTION_GEAR_PER_REVOLUTION_HOOD * REVOLUTION_HOOD_PER_DEGREES;

    double HOOD_DEGREES_TO_MOTOR_TICKS = REVOLUTION_HOOD_PER_DEGREES / REVOLUTION_GEAR_PER_REVOLUTION_HOOD
            / REVOLUTIONS_MOTOR_PER_REVOLUTION_OUTPUT_SHAFT / TICKS_PER_REVOLUTION_MOTOR;

    // Max amount of degrees the hood is allowed to move
    double MAX_DEGREES_HOOD = 35;
    // Min amount of degrees the hood is allowed to move
    double MIN_DEGREES_HOOD = 0;
    // How much the hood is allowed to move per call
    double hoodIncrementDegrees = 0.1;

    public HoodSubsystem() {
        hoodMotor = new CANSparkMax(12, MotorType.kBrushless);
        hoodMotor.restoreFactoryDefaults();

        hoodPID = hoodMotor.getPIDController();

        // TODO: Encoder takes in Channel A, Channel B, and reverse Direction
        hoodEncoder = hoodMotor.getEncoder();

        // Sets the P to 0.1
        hoodPID.setP(0.1d);
        // Sets the I to 0
        hoodPID.setI(0d);
        // Sets the D to 0
        hoodPID.setD(0d);
        // Sets the output range (the min ad max the PID is allowed to go) to -1.5V, 1.5V
        hoodPID.setOutputRange(-0.5d, 0.5d);

        // Sets the position to zero on construct
        hoodEncoder.setPosition(0);

    }

    public void periodic() {
        // Sets the hood position in ROTATIONS OF THE MOTOR!
        // ie 20 hood position = rotation of

        hoodPID.setReference(hoodPos, ControlType.kPosition);

    }

    public double getHoodPosition() {
        return hoodEncoder.getPosition();
    }

    public void moveHoodUp() {
        // Checks if the current degrees is less than the max degrees
        if (hoodPos < MAX_DEGREES_HOOD * MOTOR_TICKS_TO_HOOD_DEGREES) {
            System.out.println(MAX_DEGREES_HOOD * MOTOR_TICKS_TO_HOOD_DEGREES);
            hoodPos += hoodIncrementDegrees;
        }
    }

    public void moveHoodDown() {
        // Checks if the current degrees is more than the min degrees
        if (hoodPos > MIN_DEGREES_HOOD * MOTOR_TICKS_TO_HOOD_DEGREES) {
            hoodPos -= hoodIncrementDegrees;
        }
    }

    public void preciseMode(boolean enabled){
        if (enabled) {
            hoodIncrementDegrees = 0.05;
        } else if (!enabled) {
            hoodIncrementDegrees = 0.1;
        }
    }


    public void zeroHood() {
        hoodPos = 0;
    }
}
