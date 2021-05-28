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

import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInLayouts;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInWidgets;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardLayout;
import frc.bionic.Conversion;

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
public abstract class AbstractYawEncoder {

    protected String encoderName = "";
    private NetworkTableEntry sb_encoder_offset;
    private NetworkTableEntry sb_apply_zero_encoder_offset;

    // Offset to zero position, provided to constructor (possibly overridden by
    // dashboard)
    private double encoderOffset = 0.0;

    public AbstractYawEncoder(String encoderName) {
        this.encoderName = encoderName;
        initShuffleboard();
    }

    /**
     * Function that should be called periodically, typically from the using swerve
     * module's `periodic` function.
     */
    void periodic() {
        syncShuffleboard();
    }

    /**
     * For a relative encoder, this resets the zero point so that distances are
     * measured from the current encoder value. For implementations using an
     * absolute encoder, this function should be left unimplemented so that the
     * default no-op method provided here is used.
     */
    void setZero() {
        // no-op, by default; overridden for implementations with a
        // relative encoder.
    }

    /**
     * Return the distance from the zero point, in degrees
     *
     * @return The distance from the zero point, in degrees with offset compensation
     */
    public double getHeadingDegrees() {
        return Conversion.normalize(getRawHeadingDegrees() + encoderOffset, -180, 180);
    }

    /**
     * Return the distance from the zero point, in degrees
     *
     * @return The distance from the zero point, in degrees without offset
     *         compensation
     */
    abstract double getRawHeadingDegrees();

    /**
     * Return the distance from the zero point, in radians
     *
     * @return The distance from the zero point, in radians
     */
    double getHeadintRadians() {
        return frc.bionic.Conversion.degreesToRadians(getHeadingDegrees());
    }

    /**
     * Get the name of this encoder.
     * 
     * @return one of: "LF", "RF", "LR", "RR"
     */
    public String name() {
        return this.encoderName;
    }

    /**
     * Initialize the shuffleboard interface for this motor
     */
    private void initShuffleboard() {
        int row = 0;
        int column = 0;

        // Figure out which row/column to place the encoder offset layout
        if (encoderName.equals("LF")) {
            column = 0;
        } else if (encoderName.equals("RF")) {
            column = 2;
        } else if (encoderName.equals("LR")) {
            column = 4;
        } else if (encoderName.equals("RR")) {
            column = 6;
        }
        String fullName = "Encoder " + encoderName;

        var tab = Shuffleboard.getTab("Encoder Setup");
        ShuffleboardLayout layout = tab.getLayout(fullName, BuiltInLayouts.kGrid).withSize(2, 2)
                .withPosition(column, row).withProperties(Map.of("Label position", "HIDDEN"));

        sb_encoder_offset = layout.addPersistent(fullName + " offset", 0.0).withPosition(0, 0).getEntry();

        sb_apply_zero_encoder_offset = layout.add("Zero!", false).withWidget(BuiltInWidgets.kToggleButton)
                .withPosition(0, 1).getEntry();
    }

    /**
     * Update dynamic values on shuffleboard, and read values and reset based on
     * read values, any settable parameters.
     */
    private void syncShuffleboard() {
        // allow zeroing rotation at current module position
        if (sb_apply_zero_encoder_offset.getBoolean(false)) {
            // reset pushbutton so it's ready for additional user changes
            sb_apply_zero_encoder_offset.setBoolean(false);
            encoderOffset = -getHeadingDegrees(); // negative of distance is offset to zero
            sb_encoder_offset.setDouble(encoderOffset);
        }

        // Set encoder offset according to shuffleboard field
        encoderOffset = sb_encoder_offset.getDouble(0.0);
    }

}
