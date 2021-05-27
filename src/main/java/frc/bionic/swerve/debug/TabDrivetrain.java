package frc.bionic.swerve.debug;

import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.wpilibj.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.shuffleboard.*;
import frc.bionic.swerve.AbstractDrivetrain;

public class TabDrivetrain {

    private AbstractDrivetrain drivetrain;
    
    private NetworkTableEntry sb_lf_heading, sb_lf_speed;
	private NetworkTableEntry sb_rf_heading, sb_rf_speed;
	private NetworkTableEntry sb_gyro_reset, sb_gyroHeading, sb_override, sb_use_slew;
	private NetworkTableEntry sb_xSpeed, sb_ySpeed, sb_zSpeed;
	private NetworkTableEntry sb_lr_heading, sb_lr_speed;
	private NetworkTableEntry sb_rr_heading, sb_rr_speed;

	TabDrivetrain(AbstractDrivetrain drivetrain) {
        this.drivetrain = drivetrain;
        
		ShuffleboardTab dtTab = Shuffleboard.getTab("Drivetrain");
		int row = 0;
		dtTab.getLayout("Front", BuiltInLayouts.kGrid).withSize(2, 1).withPosition(4, row);

		sb_lf_heading = dtTab.add("LF Heading", 0).withSize(2, 2).withPosition(0, row).getEntry();
		sb_lf_speed = dtTab.add("LF Speed", 0).withSize(2, 2).withPosition(2, row).getEntry();

		sb_rf_heading = dtTab.add("RF Heading", 0).withSize(2, 2).withPosition(6, row).getEntry();
		sb_rf_speed = dtTab.add("RF Speed", 0).withSize(2, 2).withPosition(8, row).getEntry();

		row = 2;
		sb_gyro_reset = dtTab.add("Gyro Reset", false).withSize(2, 2).withPosition(0, row)
				.withWidget(BuiltInWidgets.kToggleButton).getEntry();
		sb_override = dtTab.add("Use Overrides", false).withSize(2, 2).withPosition(4, row)
				.withWidget(BuiltInWidgets.kToggleButton).getEntry();
		sb_use_slew = dtTab.add("Enable Slew", true).withSize(2, 2).withPosition(8, row)
				.withWidget(BuiltInWidgets.kToggleButton).getEntry();

		row = 4;
		sb_xSpeed = dtTab.add("xSpeed Override", 0).withSize(3, 2).withPosition(0, row)
				.withWidget(BuiltInWidgets.kNumberSlider).getEntry();
		sb_ySpeed = dtTab.add("ySpeed Override", 0).withSize(4, 2).withPosition(3, row)
				.withWidget(BuiltInWidgets.kNumberSlider).getEntry();
		sb_zSpeed = dtTab.add("zSpeed Override", 0).withSize(3, 2).withPosition(7, row)
				.withWidget(BuiltInWidgets.kNumberSlider).getEntry();

		row = 6;
		sb_lr_heading = dtTab.add("LR Heading", 0).withSize(2, 2).withPosition(0, row).getEntry();
		sb_lr_speed = dtTab.add("LR Speed", 0).withSize(2, 2).withPosition(2, row).getEntry();
		sb_gyroHeading = dtTab.add("Gyro Heading", 0).withSize(2, 2).withPosition(4, row).getEntry();
		sb_rr_heading = dtTab.add("RR Heading", 0).withSize(2, 2).withPosition(6, row).getEntry();
		sb_rr_speed = dtTab.add("RR Speed", 0).withSize(2, 2).withPosition(8, row).getEntry();
	}

	void perodic() {

		if (shouldGyroReset()) {
			drivetrain.resetGyroAngle();
		}

		SwerveModuleState[] states = drivetrain.getSwerveModuleStates();

		// Update display
		// RF, LF, LR, RR
		sb_rf_heading.setDouble(states[0].angle.getDegrees());
		sb_rf_speed.setDouble(states[0].speedMetersPerSecond);

		sb_lf_heading.setDouble(states[1].angle.getDegrees());
		sb_lf_speed.setDouble(states[1].speedMetersPerSecond);

		sb_lr_heading.setDouble(states[2].angle.getDegrees());
		sb_lr_speed.setDouble(states[2].speedMetersPerSecond);

		sb_rr_heading.setDouble(states[3].angle.getDegrees());
		sb_rr_speed.setDouble(states[3].speedMetersPerSecond);

		sb_gyroHeading.setDouble(drivetrain.getGyroAngle());
	}

    public boolean useSlew() {
		return sb_use_slew.getBoolean(false);
	}

	public boolean useDTOverride() {
		return sb_override.getBoolean(false);
	}

	public double getxSpeedOverride() {
		return sb_xSpeed.getDouble(0);
	}

	public double getySpeedOverride() {
		return sb_ySpeed.getDouble(0);
	}

	public double getzSpeedOverride() {
		return sb_zSpeed.getDouble(0);
	}

	public boolean shouldGyroReset() {
		var rst = sb_gyro_reset.getBoolean(false);
		if (rst) {
			sb_gyro_reset.setBoolean(false); // make this a momentary button
		}
		return rst;
	}
    
}
