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

import edu.wpi.first.wpilibj.Joystick;
import frc.bionic.swerve.AbstractDrivetrain;
import frc.bionic.swerve.command.DriveWithJoystick;

public class UserInterface {

    /**
     * Create the user interface operated via Joystick 0
     */
    public static void createUIJoystick0(AbstractDrivetrain drivetrain) {
        Joystick joystick0 = new Joystick(0);

        // Set the default command
        drivetrain.setDefaultCommand(new DriveWithJoystick(drivetrain, joystick0));

        // Add a mapping to the primary joystick, to lock the swerve
        // module rotation in place
        // new JoystickButton(joystick0, 4) // @todo put this back to 11
        //         .whileHeld(() -> drivetrain.lockInPlace(), drivetrain);
    }

    // public static SequentialCommandGroup followTrajectory() {
    //     UserInterfaceElement<AbstractDrivetrain> drivetrainElem = objectRegistry.get("Drivetrain");
    //     AbstractDrivetrain drivetrain = drivetrainElem.get();
    //     return new TrajectoryFollow().getTrajectoryCommand(drivetrain, "paths/flower.json");
    // }

}
