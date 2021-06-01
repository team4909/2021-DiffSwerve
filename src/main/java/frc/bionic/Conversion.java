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

package frc.bionic;

public class Conversion
{
  /**
   * Convert inches to meters
   *
   * @param inches
   *   Value to convert
   *
   * @return
   *   Converted value
   */
  static public double inchesToMeters(double inches)
  {
    return inches * 0.0254;
  }

  /**
   * Convert meters to inches
   *
   * @param meters
   *   Value to convert
   *
   * @return
   *   Converted value
   */
  static public double metersToInches(double meters)
  {
    return meters * 39.3701;
  }

  /**
   * Convert wheel revolutions to meters traveled
   *
   * @param revers
   *   Number of revolutions traveled
   *
   * @param wheelDiameterMeters
   *   The diameter of the wheel
   *
   * @return
   *   Equivalent number of meters traveled
   */
  static public double wheelRevsToMetersTraveled(double revs, double wheelDiameterMeters)
  {
    double          circumferenceMeters = Math.PI * wheelDiameterMeters;
    return circumferenceMeters * revs;
  }

  /**
   * Convert wheel meters to revolutions traveled
   *
   * @param meters
   *   Number of meters traveled
   *
   * @param wheelDiameterMeters
   *   The diameter of the wheel
   *
   * @return
   *   Equivalent number of revolutions traveled
   */
  static public double wheelMetersTraveledToRevs(double meters, double wheelDiameterMeters)
  {
    double          circumferenceMeters = Math.PI * wheelDiameterMeters;
    return meters / circumferenceMeters;
  }

  /**
   * Convert revolutions per minute to meters per second
   *
   * @param rpm
   *   Revolutions per minute to convert
   *
   * @param wheelDiameterMeters
   *   The diameter of the wheel
   *
   * @return
   *   Revolutions per minute converted into meters per second
   */
  static public double rpmToMps(double rpm, double wheelDiameterMeters)
  {
    double          circumferenceMeters = Math.PI * wheelDiameterMeters;
    return rpm * circumferenceMeters / 60;
  }

  /**
   * Convert meters per second to revolutions per minute
   *
   * @param meters
   *   Meters per second to convert
   *
   * @param wheelDiameterMeters
   *   The diameter of the wheel
   *
   * @return
   *   Meters per second converted into revolutions per minute
   */
  static public double mpsToRpm(double mps, double wheelDiameterMeters)
  {
    double          circumferenceMeters = Math.PI * wheelDiameterMeters;
    return mps * 60 / circumferenceMeters;
  }

  /**
   * Convert rotary encoder ticks to corresponding number of degrees of movement
   *
   * @param ticks
   *   Number of ticks to convert
   *
   * @param ticksPerRev
   *   Number of ticks per one full revolution of the encoder
   *
   * @return
   *   Degrees of motion resulting from specified number of ticks
   */
  static public double rotaryEncoderTicksToDegrees(double ticks, double ticksPerRev)
  {
    return ticks * (360.0 / ticksPerRev);
  }

  /**
   * Convert degrees of encoder movement to corresponding number of ticks
   *
   * @param degrees
   *   Number of degrees to convert
   *
   * @param ticksPerRev
   *   Number of ticks per one full revolution of the encoder
   *
   * @return
   *   Ticks resulting from specified degrees of motion
   */
  static public double degreesToRotaryEncoderTicks(double degrees, double ticksPerRev)
  {
    return degrees / (360.0 / ticksPerRev);
  }

  /**
   * Convert degrees to radians, first scaling degrees to to the range
   * [ 0.0, 360.0 ].
   *
   * @param degrees
   *   The number of degrees to convert to radians
   *
   * @return
   *   The degrees converted to radians
   */
  static public double degreesToRadians(double degrees)
  {
    // Normalize provided degrees to be within [0.0, 360.0]
    degrees = normalize(degrees, 0.0, 360.0);
    return (2 * Math.PI) * (degrees / 360);
  }

  /**
   * Convert radians to degrees, first scaling radians to the range
   * [ 0.0, 2 * PI ].
   *
   * @param radians
   *   The number of radians to convert to degrees
   *
   * @return
   *   The radians converted to degrees
   */
  static public double radiansToDegrees(double radians)
  {
    // Normalize provided radians to be in range [0.0, 2 * PI]
    radians = normalize(radians, 0.0, 2 * Math.PI);
    return 360 * radians / 2 * Math.PI;
  }

  /**
   * Normalize a value to be within a range
   * https://stackoverflow.com/a/2021986
   *
   * @param value
   *   The value to be normalized
   *
   * @param rangeStart
   *   The beginning of the range
   *
   * @param rangeEnd
   *   The end of the range
   *
   * @return
   *   The normalized value
   */
  static public double normalize(double value, double rangeStart, double rangeEnd)
  {
    final double              width = rangeEnd - rangeStart;
    final double              offsetValue = value - rangeStart; // value relative to 0

    return (offsetValue - (Math.floor(offsetValue / width) * width)) + rangeStart;
  }

  /**
     * Convert feet per second to meters per second
     * 
     * @param fps feet per second
     * @return meters per second
     */
    static public double fpsToMps(double fps) {
      return fps * 0.3048;
  }

  /**
   * Convert meters per second to feet per second
   * 
   * @param mps meters per second
   * @return feet per second
   */
  static public double mpsToFps(double mps) {
      return mps * 3.28084;
  }
}
