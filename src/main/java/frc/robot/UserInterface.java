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
import edu.wpi.first.wpilibj.geometry.Pose2d;
import edu.wpi.first.wpilibj.geometry.Rotation2d;
import edu.wpi.first.wpilibj.geometry.Translation2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.trajectory.TrajectoryConfig;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import frc.peyton.Drivetrain;
import frc.bionic.Conversion;
import frc.bionic.TrajectoryUtils;
import frc.bionic.UserInterfaceElement;
import frc.bionic.swerve.command.DriveWithJoystick;

@SuppressWarnings( { "rawtypes", "unchecked" })
public class UserInterface
{
  // Registry of objects that our user interface can operate on
  private static HashMap<String, UserInterfaceElement> objectRegistry = new HashMap<String, UserInterfaceElement>();

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

  /**
   * Create the default user interface. All required objects are expected to
   * have been registered before this function is called.
   */
  public static void createDefaultUI()
  {
    createUIJoystick0();
    createUIJoystick1();
    createUIDashboard();
  }

  /**
   * Create the user interface operated via Joystick 0
   */
  private static void createUIJoystick0()
  {
    Joystick                           joystick0 = new Joystick(0);
    UserInterfaceElement<Drivetrain>   drivetrainElem = objectRegistry.get("Drivetrain");
    Drivetrain                         drivetrain = drivetrainElem.get();

    // Set the default command
    drivetrain.setDefaultCommand(new DriveWithJoystick(drivetrain, joystick0));

    // Add a mapping to the primary joystick, to lock the swerve
    // module rotation in place
    new JoystickButton(joystick0, 11)
      .whileHeld(() -> drivetrain.lockInPlace(), drivetrain);
  }

  public static void runTrajectory(){
    UserInterfaceElement<Drivetrain>   drivetrainElem = objectRegistry.get("Drivetrain");
    Drivetrain                         drivetrain = drivetrainElem.get();
    Rotation2d angle = Rotation2d.fromDegrees(drivetrain.getGyroAngle()); 
    SmartDashboard.putBoolean("Supply Running", false);
    SmartDashboard.putBoolean("Generate Trajectory Running", false);
    SmartDashboard.putBoolean("Swerve Controller Command Running", false);
    TrajectoryUtils.supply(new TrajectoryUtils().generateTrajectory((new Pose2d(0, 0, angle)), 
                                                                     new Pose2d(0, 144, angle), 
                                                                     new TrajectoryConfig(Conversion.inchesToMeters(144), 
                                                                     Conversion.inchesToMeters(144)), 
                                                                     drivetrain,
                                                                     new Translation2d(72, angle)),
                          drivetrain);
  }

  /**
   * Create the user interface operated via Joystick 1
   */
  private static void createUIJoystick1()
  {
    // nothing yet
  }

  /**
   * Create the user interface operated via a dashboard, e.g., Shuffleboard
   */
  private static void createUIDashboard()
  {
    // nothing yet
  }
}
