package frc.bionic.swerve.debug;

import java.util.Map;

import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.wpilibj.shuffleboard.*;
import frc.bionic.swerve.AbstractDrivetrain;

public class TabModule {

    private AbstractDrivetrain drivetrain;
    
    private NetworkTableEntry sb_moduleControl;
    private NetworkTableEntry sb_modLF, sb_modRF, sb_modLR, sb_modRR;
	private NetworkTableEntry sb_yaw_apply, sb_yaw_kP, sb_yaw_kI, sb_yaw_kIz;
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
			moduleTab.add("Selected Module", "").withSize(2, 2).withPosition(4, row).getEntry();
		}

		// Yaw Setpoints
		{
			// 0, 45, 90, 135, 180, -90, text, apply
			var setpoints = moduleTab.getLayout("Yaw Setpoints", BuiltInLayouts.kGrid);
			setpoints.withProperties(Map.of("Number of columns", 4, "Number of rows", 2, "Label position", "HIDDEN"));
			setpoints.withSize(6, 2);
			setpoints.withPosition(6, row);

			setpoints.add("0", false).withSize(1, 1).withPosition(0, 0).withWidget(BuiltInWidgets.kToggleButton)
					.getEntry();
			setpoints.add("45", false).withSize(1, 1).withPosition(1, 0).withWidget(BuiltInWidgets.kToggleButton)
					.getEntry();
			setpoints.add("90", false).withSize(1, 1).withPosition(2, 0).withWidget(BuiltInWidgets.kToggleButton)
					.getEntry();
			setpoints.add("135", false).withSize(1, 1).withPosition(3, 0).withWidget(BuiltInWidgets.kToggleButton)
					.getEntry();

			setpoints.add("180", false).withSize(1, 1).withPosition(0, 1).withWidget(BuiltInWidgets.kToggleButton)
					.getEntry();
			setpoints.add("-90", false).withSize(1, 1).withPosition(1, 1).withWidget(BuiltInWidgets.kToggleButton)
					.getEntry();
			setpoints.add("input", 3000).withSize(1, 1).withPosition(2, 1).getEntry();
			setpoints.add("Apply", false).withSize(1, 1).withPosition(3, 1).withWidget(BuiltInWidgets.kToggleButton)
					.getEntry();
		}

		row = 2;
		// graphs
		{
			moduleTab.add("Yaw Heading", 0).withSize(6, 5).withPosition(0, row).withWidget(BuiltInWidgets.kGraph)
					.getEntry();
			moduleTab.add("Yaw Error", 0).withSize(6, 5).withPosition(6, row).withWidget(BuiltInWidgets.kGraph)
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

			sb_yaw_apply = motorPidOuter.add("Apply", false).withSize(1, 1).withPosition(0, 0)
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

	}
    
}
