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

package frc.bionic.swerve;

/**
 * The Drivetrain subsystem should implement both Subsystem and this
 * interface. This one defines the additional methods that must be implemented
 * in a Drivetrain class.
 */
public interface IDrivetrainSubsystem {

  /**
   * Method to drive the robot using joystick info.
   *
   * @param xSpeed Speed of the robot in the x direction (forward) on the range [-1, 1].
   * @param ySpeed Speed of the robot in the y direction (sideways) on the range [-1, 1].
   * @param rot    Angular rate of the robot on the range [-1, 1].
   */
  void drive(double xSpeed, double ySpeed, double rot);

  /**
   * Stop all motion, translation and rotation.
   */
  default void stop() {
    drive(0, 0, 0);
  }

  /**
   * To defend a position and make the robot hard to push 
   * move modules to opposing 45 degree angles.
   */
  void lockInPlace();

  /**
   * Reset the encoders so that zero is at the current position. This is
   * a rudimentary calibration facility.
   */
  void resetEncoders();
}
