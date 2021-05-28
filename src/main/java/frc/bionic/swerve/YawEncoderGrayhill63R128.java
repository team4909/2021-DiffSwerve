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

import edu.wpi.first.wpilibj.CounterBase.EncodingType;
import edu.wpi.first.wpilibj.Encoder;

public class YawEncoderGrayhill63R128 extends AbstractYawEncoder {
    // Devices, Sensors, Actuators
    private Encoder encoder;

    /**
     * Constructor for a Grayhill 63R128 encoder implementation
     *
     * @param dioChannelA         The first of two digital IO channels to which this
     *                            encoder is connected
     *
     * @param dioChannelB         The second of two digital IO channels to which
     *                            this encoder is connected
     *
     * @param name                Name used for debugging and for shuffleboard
     *                            fields
     *
     * @param shuffleboardTabName Tab on which to place shuffleboard fields
     */
    public YawEncoderGrayhill63R128(int dioChannelA, int dioChannelB, String name, String shuffleboardTabName) {
        super(name);

        // Get access to the encoder
        encoder = new Encoder(dioChannelA, dioChannelB, true, EncodingType.k4X);

        // We want encoder "distance" in terms degrees difference from the
        // 0 point. This model Grayhill encoder yields 128 ticks per
        // revolution (non-configurable). We can therefore calculate the
        // rotation per tick, in degrees.
        encoder.setDistancePerPulse(360.0 / 128.0);
    }

    @Override
    public void setZero() {
        encoder.reset();
    }

    @Override
    double getRawHeadingDegrees() {
        return encoder.getDistance();
    }

}
