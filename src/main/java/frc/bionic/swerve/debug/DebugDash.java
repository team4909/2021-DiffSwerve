package frc.bionic.swerve.debug;

import frc.bionic.swerve.AbstractDrivetrain;

public class DebugDash {

	public TabMotor motorTab;
	public TabModule modTab;
	public TabDrivetrain dtTab;

	public DebugDash(AbstractDrivetrain drivetrain) {
		motorTab = new TabMotor(drivetrain);
		modTab = new TabModule(drivetrain);
		dtTab = new TabDrivetrain(drivetrain);
	}

	public void periodic() {
		motorTab.periodic();
		modTab.periodic();
		dtTab.periodic();
	}
}
