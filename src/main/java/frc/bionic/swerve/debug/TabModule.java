package frc.bionic.swerve.debug;

import java.util.Map;

import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.wpilibj.geometry.Rotation2d;
import edu.wpi.first.wpilibj.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.shuffleboard.*;
import frc.bionic.swerve.AbstractDrivetrain;
import frc.bionic.swerve.AbstractSwerveModule;

public class TabModule {

    private AbstractDrivetrain drivetrain;
    
    private NetworkTableEntry sb_moduleControl;
    private NetworkTableEntry sb_modLF, sb_modRF, sb_modLR, sb_modRR;
	private NetworkTableEntry sb_sel_module;
	private NetworkTableEntry sb_sp_0, sb_sp_45, sb_sp_90, sb_sp_135, sb_sp_180, sb_sp_n90, sb_sp_input, sb_sp_Apply;
	private NetworkTableEntry sb_tSP_0, sb_tSP_100, sb_tSP_500, sb_tSP_1000, sb_tSP_1500, sb_tSP_2000, sb_tSP_input, sb_tSP_Apply;
	private NetworkTableEntry sb_g_yawHeading, sb_g_yawErr;
	private NetworkTableEntry sb_yaw_pid_apply, sb_yaw_kP, sb_yaw_kI, sb_yaw_kIz;
	private NetworkTableEntry sb_yaw_kD, sb_yaw_kF, sb_yaw_max, sb_yaw_min;

	TabModule(AbstractDrivetrain drivetrain) {
        this.drivetrain = drivetrain;
		int row = 0;
		var moduleTab = Shuffleboard.getTab("Module");

		{
			sb_moduleControl = moduleTab.add("Enable Control", false).withSize(2, 2).withPosition(0, 0)
					.withWidget(BuiltInWidgets.kToggleButton).getEntry();
		}
		// Module selector
		{
			var moduleSelector = moduleTab.getLayout("Module Selector", BuiltInLayouts.kGrid);
			moduleSelector
					.withProperties(Map.of("Number of columns", 2, "Number of rows", 2, "Label position", "HIDDEN"));
			moduleSelector.withSize(2, 2);
			moduleSelector.withPosition(2, row);

			sb_modLF = moduleSelector.add("LF", false).withSize(1, 1).withPosition(0, 0)
					.withWidget(BuiltInWidgets.kToggleButton).getEntry();
			sb_modRF = moduleSelector.add("RF", false).withSize(1, 1).withPosition(1, 0)
					.withWidget(BuiltInWidgets.kToggleButton).getEntry();
			sb_modLR = moduleSelector.add("LR", false).withSize(1, 1).withPosition(0, 1)
					.withWidget(BuiltInWidgets.kToggleButton).getEntry();
			sb_modRR = moduleSelector.add("RR", false).withSize(1, 1).withPosition(1, 1)
					.withWidget(BuiltInWidgets.kToggleButton).getEntry();
		}

		{
			sb_sel_module = moduleTab.add("Selected Module", "").withSize(2, 2).withPosition(4, row).getEntry();
		}

		// Yaw Setpoints
		{
			// 0, 45, 90, 135, 180, -90, text, apply
			var setpoints = moduleTab.getLayout("Yaw Setpoints", BuiltInLayouts.kGrid);
			setpoints.withProperties(Map.of("Number of columns", 4, "Number of rows", 2, "Label position", "HIDDEN"));
			setpoints.withSize(6, 2);
			setpoints.withPosition(6, row);

			sb_sp_0 = setpoints.add("0", false).withSize(1, 1).withPosition(0, 0).withWidget(BuiltInWidgets.kToggleButton)
					.getEntry();
			sb_sp_45 = setpoints.add("45", false).withSize(1, 1).withPosition(1, 0).withWidget(BuiltInWidgets.kToggleButton)
					.getEntry();
			sb_sp_90 = setpoints.add("90", false).withSize(1, 1).withPosition(2, 0).withWidget(BuiltInWidgets.kToggleButton)
					.getEntry();
			sb_sp_135 = setpoints.add("135", false).withSize(1, 1).withPosition(3, 0).withWidget(BuiltInWidgets.kToggleButton)
					.getEntry();

			sb_sp_180 = setpoints.add("180", false).withSize(1, 1).withPosition(0, 1).withWidget(BuiltInWidgets.kToggleButton)
					.getEntry();
			sb_sp_n90 = setpoints.add("-90", false).withSize(1, 1).withPosition(1, 1).withWidget(BuiltInWidgets.kToggleButton)
					.getEntry();
			sb_sp_input = setpoints.add("input", -45).withSize(1, 1).withPosition(2, 1).getEntry();
			sb_sp_Apply = setpoints.add("Apply", false).withSize(1, 1).withPosition(3, 1).withWidget(BuiltInWidgets.kToggleButton)
					.getEntry();
		}

		// Translation Setpoints
		{
			//0, 100, 500, 1000, 1500, 2000, text, apply
			var setpoints = moduleTab.getLayout("Translation Setpoints", BuiltInLayouts.kGrid);
			setpoints.withProperties(Map.of("Number of columns", 4, "Number of rows", 2, "Label position", "HIDDEN"));
			setpoints.withSize(6, 2);
			setpoints.withPosition(12, row);

			sb_tSP_0 = setpoints.add("0",    false).withSize(1, 1).withPosition(0, 0).withWidget(BuiltInWidgets.kToggleButton).getEntry();
			sb_tSP_100 = setpoints.add("100",  false).withSize(1, 1).withPosition(1, 0).withWidget(BuiltInWidgets.kToggleButton).getEntry();
			sb_tSP_500 = setpoints.add("500",  false).withSize(1, 1).withPosition(2, 0).withWidget(BuiltInWidgets.kToggleButton).getEntry();
			sb_tSP_1000 = setpoints.add("1000", false).withSize(1, 1).withPosition(3, 0).withWidget(BuiltInWidgets.kToggleButton).getEntry();

			sb_tSP_1500 = setpoints.add("1500",  false).withSize(1, 1).withPosition(0, 1).withWidget(BuiltInWidgets.kToggleButton).getEntry();
			sb_tSP_2000 = setpoints.add("2000",  false).withSize(1, 1).withPosition(1, 1).withWidget(BuiltInWidgets.kToggleButton).getEntry();
			sb_tSP_input = setpoints.add("input",  3000).withSize(1, 1).withPosition(2, 1).getEntry();
			sb_tSP_Apply = setpoints.add("Apply", false).withSize(1, 1).withPosition(3, 1).withWidget(BuiltInWidgets.kToggleButton).getEntry();
		}

		row = 2;
		// graphs
		{
			sb_g_yawHeading = moduleTab.add("Yaw Heading", 0).withSize(6, 5).withPosition(0, row).withWidget(BuiltInWidgets.kGraph)
					.getEntry();
			sb_g_yawErr = moduleTab.add("Yaw Error", 0).withSize(6, 5).withPosition(6, row).withWidget(BuiltInWidgets.kGraph)
					.getEntry();
			// moduleTab.add("Translation RPM", 0).withSize(5, 5).withPosition(12,
			// row).withWidget(BuiltInWidgets.kGraph).getEntry();
		}

		// Yaw PID
		{
			// kP, kI, kIz, kD, kF, max, min, apply
			var motorPidOuter = moduleTab.getLayout("Yaw PID", BuiltInLayouts.kGrid);
			motorPidOuter
					.withProperties(Map.of("Number of columns", 1, "Number of rows", 2, "Label position", "HIDDEN"));
			motorPidOuter.withSize(3, 5);
			motorPidOuter.withPosition(12, row);

			sb_yaw_pid_apply = motorPidOuter.add("Apply", false).withSize(1, 1).withPosition(0, 0)
					.withWidget(BuiltInWidgets.kToggleButton).getEntry();

			var motorPid = motorPidOuter.getLayout("Inner", BuiltInLayouts.kGrid);
			motorPid.withProperties(Map.of("Number of columns", 2, "Number of rows", 4));
			motorPid.withPosition(0, 1);

			sb_yaw_kP = motorPid.add("kP", 0).withSize(1, 1).withPosition(0, 0).getEntry();
			sb_yaw_kI = motorPid.add("kI", 0).withSize(1, 1).withPosition(0, 1).getEntry();
			sb_yaw_kIz = motorPid.add("kIz", 0).withSize(1, 1).withPosition(0, 2).getEntry();
			sb_yaw_kD = motorPid.add("kD", 0).withSize(1, 1).withPosition(0, 3).getEntry();

			sb_yaw_kF = motorPid.add("kF", 0).withSize(1, 1).withPosition(1, 0).getEntry();
			sb_yaw_max = motorPid.add("max", 0).withSize(1, 1).withPosition(1, 1).getEntry();
			sb_yaw_min = motorPid.add("min", 0).withSize(1, 1).withPosition(1, 2).getEntry();
		}
	}

	void perodic() {
		AbstractSwerveModule selectedModule = null;
        if (sb_modLF.getBoolean(false)) {
            sb_modLF.setBoolean(false);
            sb_sel_module.setString("LF");
        }
        if (sb_modRF.getBoolean(false)) {
            sb_modRF.setBoolean(false);
            sb_sel_module.setString("RF");
        }
        if (sb_modLR.getBoolean(false)) {
            sb_modLR.setBoolean(false);
            sb_sel_module.setString("LR");
        }
        if (sb_modRR.getBoolean(false)) {
            sb_modRR.setBoolean(false);
            sb_sel_module.setString("RR");
        }
        switch (sb_sel_module.getString("")) {
            case "LF":
                selectedModule = drivetrain.swerveLF;
                break;
            case "RF":
                selectedModule = drivetrain.swerveRF;
                break;
            case "LR":
                selectedModule = drivetrain.swerveLR;
                break;
            case "RR":
                selectedModule = drivetrain.swerveRR;
                break;
        }
        if (selectedModule == null) {
            // waiting for user to make selection
            // System.out.println("Selected Module is NULL");
            return;
        }
		// update the graphs
		sb_g_yawHeading.setDouble(selectedModule.getYawHeading());
		sb_g_yawErr.setDouble(selectedModule.getYawClosedLoopError());

		// setpoints
		double goalHeading = 0;

		if (sb_sp_0.getBoolean(false)) {
			sb_sp_0.setBoolean(false); // make momentary
			goalHeading = 0;
		}
		if (sb_sp_45.getBoolean(false)) {
			sb_sp_45.setBoolean(false); // make momentary
			goalHeading = 45;
		}
		if (sb_sp_90.getBoolean(false)) {
			sb_sp_90.setBoolean(false); // make momentary
			goalHeading = 90;
		}
		if (sb_sp_135.getBoolean(false)) {
			sb_sp_135.setBoolean(false); // make momentary
			goalHeading = 135;
		}
		if (sb_sp_180.getBoolean(false)) {
			sb_sp_180.setBoolean(false); // make momentary
			goalHeading = 180;
		}
		if (sb_sp_n90.getBoolean(false)) {
			sb_sp_n90.setBoolean(false); // make momentary
			goalHeading = -90;
		}
		if (sb_sp_Apply.getBoolean(false)) {
			sb_sp_Apply.setBoolean(false); // make momentary
			goalHeading = sb_sp_input.getDouble(0);
		}

		double goalTransRPM = 0;
		if (sb_tSP_0.getBoolean(false)) {
			sb_tSP_0.setBoolean(false); // make momentary
			goalTransRPM = 0;
		}
		if (sb_tSP_100.getBoolean(false)) {
			sb_tSP_100.setBoolean(false); // make momentary
			goalTransRPM = 100;
		}
		if (sb_tSP_500.getBoolean(false)) {
			sb_tSP_500.setBoolean(false); // make momentary
			goalTransRPM = 500;
		}
		if (sb_tSP_1000.getBoolean(false)) {
			sb_tSP_1000.setBoolean(false); // make momentary
			goalTransRPM = 1000;
		}
		if (sb_tSP_1500.getBoolean(false)) {
			sb_tSP_1500.setBoolean(false); // make momentary
			goalTransRPM = 1500;
		}
		if (sb_tSP_2000.getBoolean(false)) {
			sb_tSP_2000.setBoolean(false); // make momentary
			goalTransRPM = 2000;
		}
		if (sb_tSP_Apply.getBoolean(false)) {
			sb_tSP_Apply.setBoolean(false); // make momentary
			goalTransRPM = sb_tSP_input.getDouble(0);
		}
		
		var state = new SwerveModuleState(goalTransRPM, Rotation2d.fromDegrees(goalHeading));
		selectedModule.setModuleState(state);

		// pid
        if (sb_yaw_pid_apply.getBoolean(true)) {
            selectedModule.setYawPIIzDF(
                sb_yaw_kP.getDouble(0), 
                sb_yaw_kI.getDouble(0), 
                sb_yaw_kIz.getDouble(0),
                sb_yaw_kD.getDouble(0), 
                sb_yaw_kF.getDouble(0));
			selectedModule.setYawPIDOutputRange(sb_yaw_max.getDouble(0), sb_yaw_min.getDouble(0));
        }
	}

	// This should stop all drivetrain code from sending commands directly to
    // motors to allow this class direct control.
    public boolean useModuleControl(String name) {
		if (name == sb_sel_module.getString("")) {
			System.out.println("Name is: "+name);
        	return sb_moduleControl.getBoolean(false);
		}
		return false;
    }
    
}
