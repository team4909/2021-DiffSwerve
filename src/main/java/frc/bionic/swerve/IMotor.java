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
 *
 * The constructor might include (but not be limited to) the following:
 *
 *   - instantiate an appropriate motor controller instance
 *   - set factory defaults
 *   - set initial motor PID values
 */
public interface IMotor
{
  /**
   * Function that should be called periodically, typically from
   * the using swerve module's `periodic` function.
   */
  default void periodic()
  {
    // no-op
  }

  /**
   * Set the PID setpoint, in revolutions per minute
   *
   * @param setpointRPM
   *   The requested RPM which the PID attempts to attain
   */
  void setGoalRPM(double goalRPM);

  /**
   * Get the current encoder-ascertained velocity of the motor, in RPM
   */
  public double getVelocityRPM();

  /**
   * Get the error as calculated by the PID controller
   * @return pid error
   */
  public double getClosedLoopError();

  public void setPIIzDF(double kp, double kI, double kIz, double kD, double kF);

  public void setOutputRange(double max, double min);
}
