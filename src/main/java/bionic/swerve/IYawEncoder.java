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

/*
 * This is the interface expected to be implemented for the encoder that
 * ascertains the yaw of the swerve module to be used with Bionic
 * Swerve.
 *
 * The constructor might include (but not be limited to) the following:
 *
 *   - instantiate an encoder instance
 *   - set factory defaults
 *   - set zero point, if not an absolute encoder
 *   - initialize the yaw PID
 *   - set yaw PID's continuous output range
 */
public interface IYawEncoder
{
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
    return bionic.Conversion.degreesToRadians(getDistanceDegrees());
  }
}
