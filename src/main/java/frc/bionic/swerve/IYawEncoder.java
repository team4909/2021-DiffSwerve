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

/*
 * This is the interface expected to be implemented for the encoder that
 * ascertains the yaw of the swerve module to be used with Bionic
 * Swerve.
 *
 * The constructor might include (but not be limited to) the following:
 *
 *   - instantiate an encoder instance
 *   - set factory defaults
 *   - instantiate and initialize the yaw PID
 *   - set yaw PID's continuous output range
 */
public interface IYawEncoder
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
   * For a relative encoder, this resets the zero point so that distances are
   * measured from the current encoder value. For implementations using an
   * absolute encoder, this function should be left unimplemented so that the
   * default no-op method provided here is used.
   */
  default void setZero()
  {
    // no-op, by default; overridden for implementations with a
    // relative encoder.
  }

  /**
   * Return the distance from the zero point, in degrees
   *
   * @return
   *   The distance from the zero point, in degrees
   */
  double getDistanceDegrees();

  /**
   * Return the distance from the zero point, in radians
   *
   * @return
   *   The distance from the zero point, in radians
   */
  default double getDistanceRadians()
  {
    return frc.bionic.Conversion.degreesToRadians(getDistanceDegrees());
  }

  /**
   * Set the goal point in degrees
   *
   * @param goal
   *   The requested goal, in degrees, to attempt to attain
   */
  void setGoalDegrees(double goal);

  /**
   * Calculate output based on previous goal provided to
   * `setGoalDegrees`, given the current condition.
   *
   * @return
   *   The calculated output, in range [-1.0, 1.0].
   */
  double getOutputSignedPercent();

  /**
   * Calculate output based on previous goal provided to
   * `setGoalDegrees`, given the current condition.
   *
   * @param goal
   *   A new goal, in degrees, to attempt to attain
   *
   * @return
   *   The calculated output, in range [-1.0, 1.0].
   */
  double getOutputSignedPercent(double goalDegrees);
}
