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

import edu.wpi.first.wpilibj.geometry.Pose2d;
import edu.wpi.first.wpilibj.geometry.Rotation2d;
import edu.wpi.first.wpilibj.geometry.Translation2d;
import edu.wpi.first.wpilibj.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.kinematics.SwerveDriveKinematics;
import edu.wpi.first.wpilibj.kinematics.SwerveDriveOdometry;
import edu.wpi.first.wpilibj.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public abstract class AbstractDrivetrain extends SubsystemBase {
  private AbstractSwerveModule    swerveRF; // right front
  private AbstractSwerveModule    swerveLF; // left front
  private AbstractSwerveModule    swerveLR; // left rear
  private AbstractSwerveModule    swerveRR; // right rear

  private SwerveDriveKinematics   kinematics;
  private SwerveDriveOdometry     odometry;
  private Pose2d                  currentPose;

  private double                  kMaxSpeed = 2.0;  // meters per second
  public SwerveModuleState[] swerveModuleStates;


  /**
   * Extending class must implement `getGyroAngle` method which returns the
   * robot's rotation in degrees. Degrees can be continuous, so result may be
   * outside range of [0.0, 360.0]. Degrees are measured *counterclockwise*
   * from zero, where zero is down-field towards opponent. Because degrees are
   * measured counterclockwise, the result may need to be negated, as devices
   * like NavX return degrees measured cockwise from zero.
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

    // Assume our current pose is 5 meters along long end of field, in
    // the center of the field along the short end, and facing forward.
     odometry = new SwerveDriveOdometry(
      kinematics,
      Rotation2d.fromDegrees(getGyroAngle()),
      new Pose2d(5.0, 13.5, new Rotation2d()));
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
    
    currentPose = odometry.update(Rotation2d.fromDegrees(getGyroAngle()), swerveRF.getModuleState(), swerveLF.getModuleState(), swerveLR.getModuleState(), swerveRR.getModuleState());
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
                                            Rotation2d.fromDegrees(this.getGyroAngle()));

    swerveModuleStates = kinematics.toSwerveModuleStates(chassisSpeeds);
    SwerveDriveKinematics.normalizeWheelSpeeds(swerveModuleStates, kMaxSpeed);
    
    actuateModules(swerveModuleStates);
  }

  public void actuateModules(SwerveModuleState[] states){
    swerveLF.setModuleState(states[0]);
    swerveRF.setModuleState(states[1]);
    swerveLR.setModuleState(states[2]);
    swerveRR.setModuleState(states[3]);
  }

  /**
   * Orient the wheels such that the robot attempts to prevent being pushed
   * around by other robots.
   */
  public void lockInPlace() {
    swerveLF.setModuleState(new SwerveModuleState(0, Rotation2d.fromDegrees(-45)));
    swerveRF.setModuleState(new SwerveModuleState(0, Rotation2d.fromDegrees(45)));
    swerveLR.setModuleState(new SwerveModuleState(0, Rotation2d.fromDegrees(45)));
    swerveRR.setModuleState(new SwerveModuleState(0, Rotation2d.fromDegrees(-45)));
  }

  //Acessor Methods

  public SwerveDriveKinematics getKinematics(){
    return kinematics;
  }

  public Pose2d getCurrentPose(){
    return currentPose;
  }

  public SwerveModuleState[] getSwerveModuleStates(){
    return swerveModuleStates;
  }


}
