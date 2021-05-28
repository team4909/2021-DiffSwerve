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

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.TalonFXControlMode;
import com.ctre.phoenix.motorcontrol.TalonFXInvertType;
import com.ctre.phoenix.motorcontrol.can.TalonFX;

/**
 * MotorFalcon500
 */
public class MotorFalcon500 implements IMotor {

    // Initial, default PID constants, may be overridden by persistent shuffleboard fields
    public static final double kMotorP = 0.1;
    public static final double kMotorI = 0.0;
    public static final double kMotorD = 3.0;
    public static final double kMotorIz = 300.0;
    public static final double kMotorFf = 0.0;
    private static final int kMotorSlot = 0;

    private TalonFX motor;

    /**
     * Constructor for Falcon500/TalonFX
     *
     * @param deviceId The CAN channel to which this motor controller is connected
     * @param bClockwise Whether to set `inverted` to clockwise or counterclockwise
     * @param name Name used for debugging and for shuffleboard fields
     * @param shuffleboardTabName Tab on which to place shuffleboard fields
     */
    public MotorFalcon500(int deviceId, boolean bClockwise, String name, String shuffleboardTabName) {

        motor = new TalonFX(deviceId);

        // Reset settings to make sure we have a clean slate
        motor.configFactoryDefault();

        // Sets motor to brake mode, to prevent forced motor rotation
        motor.setNeutralMode(NeutralMode.Brake);

        // Set direction, since one motor faces up; the other, down.
        motor.setInverted(bClockwise ? TalonFXInvertType.Clockwise : TalonFXInvertType.CounterClockwise);

        // Clear any previous setpoints... @todo may be redundant
        motor.set(ControlMode.PercentOutput, 0);

        motor.config_kP(kMotorSlot, kMotorP);
        motor.config_kI(kMotorSlot, kMotorI);
        motor.config_kD(kMotorSlot, kMotorD);
        motor.config_IntegralZone(kMotorSlot, kMotorIz);
        motor.config_kF(kMotorSlot, kMotorFf);
    }

    // interface implementation
    public void setGoalRPM(double goalRPM) {
        // Convert RPM to ticks. Velocity is set in ticks per 100ms
        double ticksPer100ms = (goalRPM / (60 * 10)) * 2048;
        // SmartDashboard.putNumber(name + " goal", ticksPer100ms);
        motor.set(TalonFXControlMode.Velocity, ticksPer100ms);
    }

    // interface implementation
    public double getVelocityRPM() {
        // Convert ticks to RPM. Velocity is internally in ticks per 100ms
        return (motor.getSelectedSensorVelocity() / 2048) * 10 * 60;
    }

    @Override
    public double getClosedLoopError() {
        return motor.getClosedLoopError();
    }

    @Override
    public void setPIIzDF(double kP, double kI, double kIz, double kD, double kF) {
        motor.config_kP(kMotorSlot, kP);
        motor.config_kI(kMotorSlot, kI);
        motor.config_kD(kMotorSlot, kD);
        motor.config_IntegralZone(kMotorSlot, kIz);
        motor.config_kF(kMotorSlot, kF);
    }

    @Override
    public void setOutputRange(double max, double min) {
        motor.configClosedLoopPeakOutput(kMotorSlot, max);
        // min is ignored as the motor does not support it
    }
}
