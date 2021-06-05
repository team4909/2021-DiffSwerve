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

import java.util.HashMap;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.GenericHID.Hand;
import edu.wpi.first.wpilibj.Joystick.AxisType;
import edu.wpi.first.wpilibj.XboxController.Axis;
import edu.wpi.first.wpilibj.controller.ControllerUtil;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
// import frc.bionic.TrajectoryFollow;
import frc.bionic.UserInterfaceElement;
import frc.bionic.swerve.AbstractDrivetrain;
import frc.bionic.swerve.command.DriveWithJoystick;
import frc.robot.subsystems.indexer.IndexerSubsystem;
import frc.robot.subsystems.shooter.HoodSubsystem;
import frc.robot.subsystems.shooter.ShooterSubsystem;

@SuppressWarnings( { "rawtypes", "unchecked" })
public class UserInterface
{
  // Registry of objects that our user interface can operate on
  private static HashMap<String, UserInterfaceElement> objectRegistry = new HashMap<String, UserInterfaceElement>();
  private static Joystick joystick0;
  private static XboxController gamepad1;

  private static IndexerSubsystem indexerSubsystem;
  private static AbstractDrivetrain drivetrain;
  private static ShooterSubsystem shooterSubsystem;
  private static HoodSubsystem hoodSubsystem;


  /**
   * Add an object to the registry
   *
   * @param id
   *   The ID to use for access to the provided object
   *
   * @param obj
   *   The object accessed via reference by the provided ID
   */
  public static void registerObject(String id, UserInterfaceElement obj)
  {
    objectRegistry.put(id, obj);
  }

//   /**
//    * Create the default user interface. All required objects are expected to
//    * have been registered before this function is called.
//    */
//   public static void createDefaultUI()
//   {
//     createUIJoystick0();
//     // createUIJoystick1();
//     // createUIDashboard();
//   }

  /**
   * Create the user interface operated via Joystick 0
   */
  public static void createUIJoystick0()
  {
    joystick0 = new Joystick(0);
    UserInterfaceElement<AbstractDrivetrain> drivetrainElement = objectRegistry.get("Drivetrain");
    drivetrain = drivetrainElement.get();

    // Set the default command
    drivetrain.setDefaultCommand(new DriveWithJoystick(drivetrain, joystick0));

    // Add a mapping to the primary joystick, to lock the swerve
    // module rotation in place
    // new JoystickButton(joystick0, 4) //@todo put this back to 11
    //   .whileHeld(() -> drivetrain.lockInPlace(), drivetrain);
    
  }

  /**
   * Create the user interface operated via Joystick 1
   * Use for operating non-driving subsystems
   */
  public static void createUIGamepad1(){
    gamepad1 = new XboxController(1);
    UserInterfaceElement<IndexerSubsystem> indexerElement = objectRegistry.get("Indexer");
    indexerSubsystem = indexerElement.get();    

    UserInterfaceElement<ShooterSubsystem> shooterElement = objectRegistry.get("Shooter");
    shooterSubsystem = shooterElement.get();

    UserInterfaceElement<HoodSubsystem> hoodElement = objectRegistry.get("Hood");
    hoodSubsystem = hoodElement.get();
  }

  public static void periodic(){

    // Checks to see if Right Trigger is pressed
    if(Math.abs(gamepad1.getTriggerAxis(Hand.kRight)) > 0.01){
      // Runs the indexer at full speed forward
      new InstantCommand(indexerSubsystem::runIndexer, indexerSubsystem).schedule();
    } else if(Math.abs(gamepad1.getTriggerAxis(Hand.kLeft)) > 0.01){
      // Runs the indexer at full speed backward
      new InstantCommand(indexerSubsystem::reverseIndexer, indexerSubsystem).schedule();
    } else {
      // If nothing is being pressed do not run indexer
      new InstantCommand(indexerSubsystem::stopIndexer, indexerSubsystem).schedule();
    }

    // Check to see if the up POV is pressed
    if(gamepad1.getPOV() == 0){
      // Moves the hood up by 10 ticks
      new InstantCommand(hoodSubsystem::moveHoodUp, hoodSubsystem).schedule();
      System.out.println("UP");
    } else if(gamepad1.getPOV() == 180){
      // Moves the hood down by 10 ticks
      new InstantCommand(hoodSubsystem::moveHoodDown, hoodSubsystem).schedule();
      System.out.println("DOWN");
    }
  }


  
//   // public static SequentialCommandGroup followTrajectory(){
//   //   UserInterfaceElement<AbstractDrivetrain>   drivetrainElem = objectRegistry.get("Drivetrain");
//   //   AbstractDrivetrain            drivetrain = drivetrainElem.get();
//   //   return new TrajectoryFollow().getTrajectoryCommand(drivetrain, "paths/flower.json");
    
//   // }

//   /**
//    * Create the user interface operated via Joystick 1
//    */
//   private static void createUIJoystick1()
//   {
//     // nothing yet
//   }

//   /**
//    * Create the user interface operated via a dashboard, e.g., Shuffleboard
//    */
//   private static void createUIDashboard()
//   {
//     // nothing yet
//   }
}
