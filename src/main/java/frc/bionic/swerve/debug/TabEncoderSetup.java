package frc.bionic.swerve.debug;

import java.util.List;
import java.util.Map;

import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInLayouts;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInWidgets;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import frc.bionic.swerve.AbstractDrivetrain;
import frc.bionic.swerve.AbstractYawEncoder;

public class TabEncoderSetup {

    private AbstractDrivetrain drivetrain;

    private NetworkTableEntry sb_encLF_offset, sb_encLF_curr, sb_encLF_zero;
    private NetworkTableEntry sb_encRF_offset, sb_encRF_curr, sb_encRF_zero;
    private NetworkTableEntry sb_encLR_offset, sb_encLR_curr, sb_encLR_zero;
    private NetworkTableEntry sb_encRR_offset, sb_encRR_curr, sb_encRR_zero;

    private double encoderLFoffsetToZero = 0;
    private double encoderRFoffsetToZero = 0;
    private double encoderLRoffsetToZero = 0;
    private double encoderRRoffsetToZero = 0;

    TabEncoderSetup(AbstractDrivetrain drivetrain) {
        this.drivetrain = drivetrain;
        
        // Figure out which row/column to place the encoder offset layout
        
        var setupTab = Shuffleboard.getTab("Encoder Setup");

        int row = 0;
        int column = 0;
        // Encoder LF
        {
            var yawEncoder = drivetrain.swerveLF.yawEncoder;
            var encoderLayout = setupTab.getLayout(yawEncoder.name(), BuiltInLayouts.kGrid).withSize(2, 2).withPosition(column, row).withProperties(Map.of("Label position", "HIDDEN"));
            sb_encLF_offset = encoderLayout.addPersistent(yawEncoder.name()+" offset", 0.0).withPosition(0, 0).getEntry();
            sb_encLF_zero = encoderLayout.add("Zero!", false).withWidget(BuiltInWidgets.kToggleButton).withPosition(0, 1).getEntry();
            sb_encLF_curr = setupTab.add(yawEncoder.name()+" Heading", 0).withSize(2, 2).withPosition(column, row+2).getEntry();
        }
        column += 2;
        // Encoder RF
        {
            var yawEncoder = drivetrain.swerveRF.yawEncoder;
            var encoderLayout = setupTab.getLayout(yawEncoder.name(), BuiltInLayouts.kGrid).withSize(2, 2).withPosition(column, row).withProperties(Map.of("Label position", "HIDDEN"));
            sb_encRF_offset = encoderLayout.addPersistent(yawEncoder.name()+" offset", 0.0).withPosition(0, 0).getEntry();
            sb_encRF_zero = encoderLayout.add("Zero!", false).withWidget(BuiltInWidgets.kToggleButton).withPosition(0, 1).getEntry();
            sb_encRF_curr = setupTab.add(yawEncoder.name()+" Heading", 0).withSize(2, 2).withPosition(column, row+2).getEntry();
        }
        column += 2;
        // Encoder LR
        {
            var yawEncoder = drivetrain.swerveLR.yawEncoder;
            var encoderLayout = setupTab.getLayout(yawEncoder.name(), BuiltInLayouts.kGrid).withSize(2, 2).withPosition(column, row).withProperties(Map.of("Label position", "HIDDEN"));
            sb_encLR_offset = encoderLayout.addPersistent(yawEncoder.name()+" offset", 0.0).withPosition(0, 0).getEntry();
            sb_encLR_zero = encoderLayout.add("Zero!", false).withWidget(BuiltInWidgets.kToggleButton).withPosition(0, 1).getEntry();
            sb_encLR_curr = setupTab.add(yawEncoder.name()+" Heading", 0).withSize(2, 2).withPosition(column, row+2).getEntry();
        }
        column += 2;
        // Encoder RR
        {
            var yawEncoder = drivetrain.swerveRR.yawEncoder;
            var encoderLayout = setupTab.getLayout(yawEncoder.name(), BuiltInLayouts.kGrid).withSize(2, 2).withPosition(column, row).withProperties(Map.of("Label position", "HIDDEN"));
            sb_encRR_offset = encoderLayout.addPersistent(yawEncoder.name()+" offset", 0.0).withPosition(0, 0).getEntry();
            sb_encRR_zero = encoderLayout.add("Zero!", false).withWidget(BuiltInWidgets.kToggleButton).withPosition(0, 1).getEntry();
            sb_encRR_curr = setupTab.add(yawEncoder.name()+" Heading", 0).withSize(2, 2).withPosition(column, row+2).getEntry();
        }


        
    }

    /**
     * Update dynamic values on shuffleboard, and read values and reset based on
     * read values, any settable parameters.
     */
    void periodic() {

        // Update Current Heading Indicator
        sb_encLF_curr.setDouble(drivetrain.swerveLF.yawEncoder.getHeadingDegrees());
        sb_encRF_curr.setDouble(drivetrain.swerveRF.yawEncoder.getHeadingDegrees());
        sb_encLR_curr.setDouble(drivetrain.swerveLR.yawEncoder.getHeadingDegrees());
        sb_encRR_curr.setDouble(drivetrain.swerveRR.yawEncoder.getHeadingDegrees());


        if (sb_encLF_zero.getBoolean(false)) {
            sb_encLF_zero.setBoolean(false); // make momentary
            encoderLFoffsetToZero = -drivetrain.swerveLF.yawEncoder.getHeadingDegrees();
            sb_encLF_offset.setDouble(encoderLFoffsetToZero);
        }
        if (sb_encRF_zero.getBoolean(false)) {
            sb_encRF_zero.setBoolean(false); // make momentary
            encoderRFoffsetToZero = -drivetrain.swerveRF.yawEncoder.getHeadingDegrees();
            sb_encRF_offset.setDouble(encoderRFoffsetToZero);
        }
        if (sb_encLR_zero.getBoolean(false)) {
            sb_encLR_zero.setBoolean(false); // make momentary
            encoderLRoffsetToZero = -drivetrain.swerveLR.yawEncoder.getHeadingDegrees();
            sb_encLR_offset.setDouble(encoderLRoffsetToZero);
        }
        if (sb_encRR_zero.getBoolean(false)) {
            sb_encRR_zero.setBoolean(false); // make momentary
            encoderRRoffsetToZero = -drivetrain.swerveRR.yawEncoder.getHeadingDegrees();
            sb_encRR_offset.setDouble(encoderRRoffsetToZero);
        }


        
        
        
        




        // allow zeroing rotation at current module position
        if (sb_save_enc_offset.getBoolean(false)) {
            // reset pushbutton so it's ready for additional user changes
            sb_save_enc_offset.setBoolean(false);
            encoderOffset = -encoder.getDistance(); // negative of distance is offset to
            // zero
            // sb_encoder_offset.setDouble(encoderOffset);
        }
        sb_encoder_current.setDouble(getHeadingDegrees());

        // Set encoder offset according to shuffleboard field
        // encoderOffset = sb_encoder_offset.getDouble(0.0);
    }

}
