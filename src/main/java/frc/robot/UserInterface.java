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

import java.lang.Object;
import java.util.HashMap;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import frc.peyton.Drivetrain;
import frc.bionic.swerve.command.DriveWithJoystick;

public class UserInterface
{
  // Registry of objects that our user interface can operate on
  private static HashMap<String, Object> objectRegistry = new HashMap<String, Object>();

  /**
   * Add an object to the registry
   *
   * @param id
   *   The ID to use for access to the provided object
   *
   * @param obj
   *   The object accessed via reference by the provided ID
   */
  public static void registerObject(String id, Object obj)
  {
    objectRegistry.put(id, obj);
  }

  /*
   * Create the default user interface. All required objects are expected to
   * have been registered before this function is called.
   */
  public static void createDefaultUI()
  {
    Joystick                  joystick0 = new Joystick(0);
    Drivetrain                drivetrain = (Drivetrain) objectRegistry.get("Drivetrain");

    if (drivetrain == null)
    {
      throw new Error("`Drivetrain` has not been registered with UserInterface");
    }

    // Set the default command
    drivetrain.setDefaultCommand(new DriveWithJoystick(drivetrain, joystick0));

    // Add a mapping to the primary joystick, to lock the swerve
    // module rotation in place
    new JoystickButton(joystick0, 11)
      .whileHeld(() -> drivetrain.lockInPlace(), drivetrain);

    // Add a mapping to the primary joystick, to reset all encoders to zero
    new JoystickButton(joystick0, 12)
      .whenPressed(() -> drivetrain.resetEncoders(), drivetrain);
  }
}
