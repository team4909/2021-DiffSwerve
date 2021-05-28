package frc.bionic.swerve.debug;

import java.util.Map;

import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.wpilibj.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInLayouts;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInWidgets;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.bionic.swerve.AbstractDrivetrain;
import frc.bionic.swerve.AbstractSwerveModule;
import frc.bionic.swerve.IMotor;
import frc.robot.Robot;

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
