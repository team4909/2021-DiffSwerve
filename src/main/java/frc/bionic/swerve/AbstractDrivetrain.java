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

import edu.wpi.first.wpilibj.geometry.Rotation2d;
import edu.wpi.first.wpilibj.geometry.Translation2d;
import edu.wpi.first.wpilibj.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.kinematics.SwerveDriveKinematics;

public abstract class AbstractDrivetrain {
  private AbstractSwerveModule    swerveRF; // right front
  private AbstractSwerveModule    swerveLF; // left front
  private AbstractSwerveModule    swerveLR; // left rear
  private AbstractSwerveModule    swerveRR; // right rear

  private SwerveDriveKinematics   kinematics;

  private double                  kMaxSpeed = 6.0;  // meters per second


  /*
   * Extending class must implement `getGyroAngle` method which returns the
   * robot's rotation in degrees
   */
  abstract public double getGyroAngle();


  public void initialize (AbstractSwerveModule swerveRF,
                          AbstractSwerveModule swerveLF,
                          AbstractSwerveModule swerveLR,
                          AbstractSwerveModule swerveRR,
                          double kHalfWheelBaseWidthInches,
                          double kHalfWheelBaseLengthInches) {

    double                  kHalfWheelBaseWidthMeters;
    double                  kHalfWheelBaseLengthMeters;
    Translation2d           frontLeftLocation;
    Translation2d           frontRightLocation;
    Translation2d           backLeftLocation;
    Translation2d           backRightLocation;

    this.swerveRF = swerveRF;
    this.swerveLF = swerveLF;
    this.swerveLR = swerveLR;
    this.swerveRR = swerveRR;
    
    kHalfWheelBaseWidthMeters = frc.bionic.Conversion.inchesToMeters(kHalfWheelBaseWidthInches);
    kHalfWheelBaseLengthMeters = frc.bionic.Conversion.inchesToMeters(kHalfWheelBaseLengthInches);

    frontLeftLocation = new Translation2d(kHalfWheelBaseWidthMeters, kHalfWheelBaseLengthMeters);
    frontRightLocation = new Translation2d(kHalfWheelBaseWidthMeters, -kHalfWheelBaseLengthMeters);
    backLeftLocation = new Translation2d(-kHalfWheelBaseWidthMeters, kHalfWheelBaseLengthMeters);
    backRightLocation = new Translation2d(-kHalfWheelBaseWidthMeters, -kHalfWheelBaseLengthMeters);

    kinematics = new SwerveDriveKinematics(frontLeftLocation, frontRightLocation, backLeftLocation, backRightLocation);
  }

  public double getMaxSpeed() {
    return kMaxSpeed;
  }

  public void setMaxSpeed(double maxSpeed) {
    kMaxSpeed = maxSpeed;
  }

  public void periodic() {
    swerveRF.periodic();
    swerveLF.periodic();
    swerveLR.periodic();
    swerveRR.periodic();
  }

  /**
   * Method to drive the robot using joystick info.
   *
   * @param xSpeed Speed of the robot in the x direction (forward).
   * @param ySpeed Speed of the robot in the y direction (sideways).
   * @param rotate Angular rate of the robot.
   */
  public void drive(double xSpeed, double ySpeed, double rotate) {
    // Scale requested speed percentage [-1, 1] to meters per second
    xSpeed *= kMaxSpeed;
    ySpeed *= kMaxSpeed;
    rotate *= kMaxSpeed;

    var chassisSpeeds = 
      ChassisSpeeds.fromFieldRelativeSpeeds(xSpeed, ySpeed, rotate, 
                                            Rotation2d.fromDegrees(-this.getGyroAngle()));

    var swerveModuleStates = kinematics.toSwerveModuleStates(chassisSpeeds);
    SwerveDriveKinematics.normalizeWheelSpeeds(swerveModuleStates, kMaxSpeed);
    
    swerveLF.setModuleState(swerveModuleStates[0]);
    swerveRF.setModuleState(swerveModuleStates[1]);
    swerveLR.setModuleState(swerveModuleStates[2]);
    swerveRR.setModuleState(swerveModuleStates[3]);
  }
}
