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

import edu.wpi.first.wpilibj.DutyCycleEncoder;
import edu.wpi.first.wpilibj.LinearFilter;

public class YawEncoderRevHex extends AbstractYawEncoder {
    // Devices, Sensors, Actuators
    private DutyCycleEncoder encoder;

    // Offset to zero position, provided to constructor (possibly overridden by
    // dashboard)
    // private double encoderOffset = 0.0;
    private LinearFilter yawDistanceAverage;

    /**
     * Constructor for a Grayhill 63R128 encoder implementation
     *
     * @param dioChannel          The first of two digital IO channels to which this
     *                            encoder is connected
     *
     * @param name                Name used for debugging and for shuffleboard
     *                            fields
     *
     * @param shuffleboardTabName Tab on which to place shuffleboard fields
     */
    public YawEncoderRevHex(int dioChannel, String name, String shuffleboardTabName) {
        super(name);

        this.yawDistanceAverage = LinearFilter.movingAverage(10);

        // Get access to the encoder
        encoder = new DutyCycleEncoder(dioChannel);

        // We want encoder "distance" in terms degrees difference from the
        // 0 point. This model Grayhill encoder yields 128 ticks per
        // revolution (non-configurable). We can therefore calculate the
        // rotation per tick, in degrees.
        encoder.setDistancePerRotation(360.0);
    }

    public double getRawHeadingDegrees() {
        double currentYawDist = this.yawDistanceAverage.calculate(encoder.getDistance());
        return currentYawDist;
    }
}
