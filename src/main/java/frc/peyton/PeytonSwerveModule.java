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

package frc.peyton;

public class PeytonSwerveModule extends frc.bionic.swerve.SwerveModule
{
  // Gear ratio of first two pairs of gears.
  // Yaw does not depend on the third pair.
  private static final double       GEAR_RATIO_12  =  (10.0/80.0) * (34.0/90.0);

  // Gear ratio of all three pairs of gears.
  // Wheel speed depends on all three.
  private static final double       GEAR_RATIO_123 =  (10.0/80.0) * (34.0/90.0) * (82.0/21.0);

  // Maximum yaw speed in RPM
  private static final double       MAX_YAW_SPEED_RPM = 200;

  public PeytonSwerveModule(int pwmMotorChannelA, int pwmMotorChannelB,
                            int dioEncoderChannelA, int dioEncoderChannelB,
                            String name, String shuffleboardTabName)
  {
    super(GEAR_RATIO_12, GEAR_RATIO_123, MAX_YAW_SPEED_RPM, 
          pwmMotorChannelA, pwmMotorChannelB,
          dioEncoderChannelA, dioEncoderChannelB,
          frc.bionic.Conversion.inchesToMeters(3.0),
          name, shuffleboardTabName);
  }
}
