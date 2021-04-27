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
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import frc.bionic.UserInterfaceElement;
import frc.bionic.swerve.command.DriveWithJoystick;
import frc.peyton.Drivetrain;

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
