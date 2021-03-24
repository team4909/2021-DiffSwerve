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

package bionic.swerve;

/**
 * This is the interface expected to be implemented for a motor / motor
 * controller to be used with Bionic Swerve. In general, each motor is
 * expected to implement a PID that keeps that motor at a specified setpoint.
 * Many motor controllers have internal PIDs, and those may be used. If the
 * motor controller being used does not have internal PIDs, the implementing
 * class should provide a software-based PID to maintain constant motor speed.
 * In either case, the PID values are provided by an external configuration,
 * and typically should not be hard-coded in the class implementing this
 * interface.
 */
public interface IMotor
{
  /*
   * The constructor might include (but not be limited to) the following:
   *
   *   - instantiate an appropriate motor controller instance
   *   - set factory defaults
   *   - set initial motor PID values
   */

  /**
   * Set the PID setpoint
   *
   * @param setpoint
   *   The requested value which the PID attempts to attain
   */
  void setPIDSetpoint(setpoint);

  /**
   * Calculate the PID output, given the current condition
   *
   * @param current
   *   The current value, as read from sensors, etc., of the process being
   *   controlled by this PID
   *
   * @return
   *   The PID output, as calculated, taking into consideration the the
   *   established kP, kI, kD, kFf, and kIZone values, and the current process
   *   value and the setpoint.
   */
  double calculate(current);
  
  /**
   * Calculate the PID output, given the current condition. The new setpoint
   * is established prior to calculating the output to be returned.
   *
   * @param current
   *   The current value, as read from sensors, etc., of the process being
   *   controlled by this PID
   *
   * @param setpoint
   *   A new setpoint to establish.
   *
   * @return
   *   The PID output, as calculated, taking into consideration the the
   *   established kP, kI, kD, kFf, and kIZone values, and the current process
   *   value and the setpoint.
   */
  double calculatePID(current, setpoint);


  /**
   * Set the output range of the motor PID
   *
   * @param min
   *   The minimum value the motor PID is allowed to output
   *
   * @param max
   *   The maximum value the motor PID is allowed to output
   */
  void setPIDOutputRange(double min, double max);

  /**
   * Set the motor PID's proportional constant
   *
   * @param kP
   *   The specified proportional constant
   */
  void setPIDkP(double kP);

  /**
   * Set the motor PID's integral constant
   *
   * @param kI
   *   The specified integral constant
   */
  void setPIDkI(double kI);

  /**
   * Set the motor PID's differential constant
   *
   * @param kD
   *   The specified differential constant
   */
  void setPIDkD(double KD);

  /**
   * Set the motor PID's feed forward constant
   *
   * @param kFf
   *   The specified feed forward constant
   */
  void setPIDkFf(double kFf);

  /**
   * Set the motor PID's integral zone constant.
   *
   * The integral zone specifies the maximum error upon which the
   * integral constant shall be used in calculating the PID's output
   *
   * @param kIZone
   *   The specified integral zone constant
   */
  void setPIDkIZone(double kIZone);

  /**
   * Set multiple constants of the motor PID
   *
   * @param kP
   *   The specified proportional constant
   *
   * @param kI
   *   The specified integral constant
   *
   * @param kD
   *   The specified differential constant
   *
   * @param kFf
   *   The specified feed forward constant
   *
   * @param kIZone
   *   The specified integral zone constant
   */
  default void setPID(double kP, double kI, double kD, double kFf, double kIzone)
  {
    this.setPIDkP(kP);
    this.setPIDkI(kI);
    this.setPIDkD(kD);
    this.setPIDkFf(kFf);
    this.setPIDkIZone(kIZone);
  }

  /**
   * Set multiple constants of the motor PID.
   *
   * @param kP
   *   The specified proportional constant
   *
   * @param kI
   *   The specified integral constant
   *
   * @param kD
   *   The specified differential constant
   *
   * @param kFf
   *   The specified feed forward constant
   */
  default void setPID(double kP, double kI, double kD, double kFf)
  {
    this.setPIDkP(kP);
    this.setPIDkI(kI);
    this.setPIDkD(kD);
    this.setPIDkFf(kFf);
  }

  /**
   * Set multiple constants of the motor PID
   *
   * @param kP
   *   The specified proportional constant
   *
   * @param kI
   *   The specified integral constant
   *
   * @param kD
   *   The specified differential constant
   */
  default void setPID(double kP, double kI, double kD)
  {
    this.setPIDkP(kP);
    this.setPIDkI(kI);
    this.setPIDkD(kD);
  }
}
