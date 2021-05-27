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
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import frc.bionic.TrajectoryFollow;
import frc.bionic.UserInterfaceElement;
import frc.bionic.swerve.AbstractDrivetrain;
import frc.bionic.swerve.Vision;

public class Robot extends TimedRobot {
  private AbstractDrivetrain drivetrain;
  private Vision vision;

  // Shuffleboard-related
  NetworkTableEntry sb_robot_type;

  @Override
  public void robotInit() {
    ShuffleboardTab tab;

    tab = Shuffleboard.getTab("Robot Setup");
    sb_robot_type = tab.addPersistent("Drivetrain Type", "Enter 'peyton' or 'team4909'").withSize(2, 1).getEntry();

    vision = new Vision(0, 0, 0, 0, 0);
  }

  @Override
  public void robotPeriodic() {
    String type;
    frc.peyton.Drivetrain drivetrainPeyton;
    frc.team4909.Drivetrain drivetrain4909;

    // If we're not yet configured with a drivetrain type...
    if (drivetrain == null) {
      // See if the user has entered a known drivetrain type
      type = sb_robot_type.getString("UNCONFIGURED");
      if (type.equals("peyton")) {
        drivetrain = drivetrainPeyton = new frc.peyton.Drivetrain();
        UserInterface.registerObject("Drivetrain", new UserInterfaceElement<frc.peyton.Drivetrain>(drivetrainPeyton));
      } else if (type.equals("team4909")) {
        drivetrain = drivetrain4909 = new frc.team4909.Drivetrain();
        UserInterface.registerObject("Drivetrain", new UserInterfaceElement<frc.team4909.Drivetrain>(drivetrain4909));
      } else {
        System.out.println("Shuffleboard's 'Robot Selection/Drivetrain Type' is invalid: " + type);
        return;
      }

      UserInterface.createDefaultUI();
    }

    drivetrain.periodic();
    CommandScheduler.getInstance().run();
  }

  @Override
  public void teleopInit() {

  }


  @Override
  public void autonomousInit() {
    // Starts the process of following a trajectory
    new TrajectoryFollow().getTrajectoryCommand(drivetrain, "paths/flower.json").schedule();
    new TrajectoryFollow().getTrajectoryCommand(drivetrain, "paths/line.json").schedule();

  }
}
