package frc.bionic.swerve;

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
   * move every module to 45 degrees.
   */
  void lockInPlace(); 
}
