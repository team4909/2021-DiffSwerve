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

package frc.robot;

import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import frc.bionic.swerve.AbstractDrivetrain;
import frc.bionic.swerve.debug.DebugDash;

public class Robot extends TimedRobot {
    private AbstractDrivetrain drivetrain;

    public static DebugDash debugDash = null;

    // Shuffleboard-related
    NetworkTableEntry sb_robot_type;

    public Robot() {
        super();

        // uncomment one or the other
        // drivetrain = new frc.peyton.Drivetrain();
        drivetrain = new frc.team4909.Drivetrain();

        UserInterface.createUIJoystick0(drivetrain);
        debugDash = new DebugDash(drivetrain);

    }

    @Override
    public void robotInit() {
    }

    @Override
    public void disabledInit() {
    }

    @Override
    public void robotPeriodic() {
        CommandScheduler.getInstance().run();

        debugDash.periodic();
    }

    @Override
    public void teleopInit() {

    }

    // @Override
    // public void autonomousInit() {
    // // Starts the process of following a trajectory
    // new TrajectoryFollow().getTrajectoryCommand(drivetrain,
    // "paths/flower.json").schedule();
    // new TrajectoryFollow().getTrajectoryCommand(drivetrain,
    // "paths/line.json").schedule();

    // }
}
