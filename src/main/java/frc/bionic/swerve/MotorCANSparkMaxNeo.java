/*
 * Team 4909, Bionics
 * Billerica Memorial High School
 *
 * Copyright:
 *   2021 Bionics
 *
 * License:
 *   MIT: https://opensource.org/licenses/MIT
 *   See the LICENSE file in the project's top-level directory for details.
 */

package frc.bionic.swerve;

import java.util.Map;

import com.revrobotics.CANEncoder;
import com.revrobotics.CANPIDController;
import com.revrobotics.CANSparkMax;
import com.revrobotics.ControlType;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import edu.wpi.first.wpilibj.MedianFilter;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInLayouts;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInWidgets;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardLayout;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class MotorCANSparkMaxNeo implements IMotor {
    // Initial, default PID constants, overridden by persistent shuffleboard fields
    private static final double kMotorP = 0.000090;
    private static final double kMotorI = 0.000001;
    private static final double kMotorD = 0.000090;
    private static final double kMotorIz = 100.0;
    private static final double kMotorFf = 0.000090;

    // To keep the motor closer to peek power, we limit the max output.
    // See torque/speed curves on https://motors.vex.com/
    private static final double kMotorMax = 0.7;
    private static final double kMotorMin = -0.7;

    // Devices, Sensors, Actuators
    private CANSparkMax motorController; // the motor controller
    private CANEncoder encoder; // internal encoder
    private CANPIDController pid; // internal PID controller
    private MedianFilter velAverage; // for displaying average RPM

    private double goalRPM = 0;

    // Shuffleboard-related
    private String name;
    // private String shuffleboardTabName;

    // NetworkTableEntry sb_vel_rpm;
    // NetworkTableEntry sb_vel_avg;
    // NetworkTableEntry sb_vel_rpm_set;
    // NetworkTableEntry sb_vel_apply;

    // NetworkTableEntry sb_pid_kp;
    // NetworkTableEntry sb_pid_ki;
    // NetworkTableEntry sb_pid_kd;
    // NetworkTableEntry sb_pid_kiz;
    // NetworkTableEntry sb_pid_kff;
    // NetworkTableEntry sb_pid_max;
    // NetworkTableEntry sb_pid_min;
    // NetworkTableEntry sb_pid_apply;

    /**
     * Constructor for a CAN Spark Max / Neo motor implementation
     *
     * @param deviceId            The CAN channel to which this motor controller is
     *                            connected
     *
     * @param name                Name used for debugging and for shuffleboard
     *                            fields
     *
     * @param shuffleboardTabName Tab on which to place shuffleboard fields
     */
    public MotorCANSparkMaxNeo(int deviceId, String name, String shuffleboardTabName) {
        // Save parameter values used elsewhere
        this.name = name;
        // this.shuffleboardTabName = shuffleboardTabName;

        // Get access to the specified motor controller
        motorController = new CANSparkMax(deviceId, MotorType.kBrushless);
        motorController.restoreFactoryDefaults();

        // Get the encoder and PID controller from the motor controller
        encoder = motorController.getEncoder();
        pid = motorController.getPIDController();

        // @todo current limits on the motor

        // Set zero RPM (motor stopped), initially
        setGoalRPM(0.0);

        // Prepare to display (on shuffleboard) recent average RPM
        velAverage = new MedianFilter(50);

        pid.setP(kMotorP);
        pid.setI(kMotorI);
        pid.setD(kMotorD);
        pid.setIZone(kMotorIz);
        pid.setFF(kMotorFf);
        pid.setOutputRange(kMotorMin, kMotorMax);

    }

    // interface implementation
    public void setGoalRPM(double goalRPM) {
        // System.out.printf("Setting Goal for (NEO) %s to %f\n", name, goalRPM);
        this.goalRPM = goalRPM;
        // SmartDashboard.putNumber(name + " goal", goalRPM);
        pid.setReference(goalRPM, ControlType.kVelocity);
    }

    // interface implementation
    public double getVelocityRPM() {
        return encoder.getVelocity();
    }

    // interface implementation
    public void periodic() {
    }


    @Override
    public double getClosedLoopError() {
        return goalRPM - getVelocityRPM();
    }

    @Override
    public void setPIIzDF(double kP, double kI, double kIz, double kD, double kF) {
        pid.setP(kP);
        pid.setI(kI);
        pid.setIZone(kIz);
        pid.setD(kD);
        pid.setFF(kF);
    }

    @Override
    public void setOutputRange(double max, double min) {
        pid.setOutputRange(max, min);
    }
}
