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
import edu.wpi.first.wpilibj.PowerDistributionPanel;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInWidgets;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
// import frc.bionic.TrajectoryFollow;
import frc.bionic.UserInterfaceElement;
import frc.bionic.swerve.AbstractDrivetrain;
import frc.bionic.swerve.debug.DebugDash;
import frc.robot.subsystems.controlpanel.ColorSensor;
import frc.robot.subsystems.controlpanel.Manipulator;
import frc.robot.subsystems.indexer.IndexerSubsystem;
import frc.robot.subsystems.shooter.HoodSubsystem;
import frc.robot.subsystems.shooter.ShooterSubsystem;

public class Robot extends TimedRobot {
  // Subsystems
  private AbstractDrivetrain drivetrain;
  private ShooterSubsystem shooter;
  private HoodSubsystem hood;
  private IndexerSubsystem indexer;
  private Manipulator manipulator;

  private PowerDistributionPanel PDP;
  private GameData gameData;
  private ColorSensor colorSensor;
  // private Vision vision
  public static DebugDash debugDash = null;

  // Shuffleboard-related
  NetworkTableEntry sb_robot_type;
  
  public Robot() {
    super();

    // uncomment one or the other
    // drivetrain = new frc.peyton.Drivetrain();
    // drivetrain = new frc.team4909.Drivetrain();
    
    shooter = new ShooterSubsystem();
    indexer = new IndexerSubsystem();
    hood = new HoodSubsystem();
    manipulator = new Manipulator(colorSensor, gameData);
    // TODO: change values based on Limelight mounting
    // vision = new Vision(0, 0, 0, 0, 0);

    PDP = new PowerDistributionPanel(0);
    // gameData = new GameData();
    // colorSensor = new ColorSensor();


    ShuffleboardTab pdp = Shuffleboard.getTab("PDP");
    pdp.add("PDP", PDP).withWidget(BuiltInWidgets.kPowerDistributionPanel).withPosition(0, 0).withSize(6, 3);

    // UserInterface.registerObject("Drivetrain", new UserInterfaceElement<AbstractDrivetrain>(drivetrain));
    UserInterface.registerObject("Shooter", new UserInterfaceElement<ShooterSubsystem>(shooter));
    UserInterface.registerObject("Hood", new UserInterfaceElement<HoodSubsystem>(hood));
    UserInterface.registerObject("Indexer", new UserInterfaceElement<IndexerSubsystem>(indexer));
    UserInterface.registerObject("Manipulator", new UserInterfaceElement<Manipulator>(manipulator));

    // UserInterface.createDefaultUI();
    // UserInterface.createUIJoystick0();
    UserInterface.createUIGamepad1();

    // debugDash = new DebugDash(drivetrain);
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
    // debugDash.periodic();
  }

  @Override
  public void teleopInit() {
  }


  @Override
  public void teleopPeriodic() {
    UserInterface.periodic();
  }

  // @Override
  // public void autonomousInit() {
  //   // Starts the process of following a trajectory
  //   new TrajectoryFollow().getTrajectoryCommand(drivetrain, "paths/flower.json").schedule();
  //   new TrajectoryFollow().getTrajectoryCommand(drivetrain, "paths/line.json").schedule();

  // }
}
