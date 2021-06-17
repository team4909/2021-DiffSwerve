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

import java.util.Map;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.wpilibj.geometry.Pose2d;
import edu.wpi.first.wpilibj.geometry.Rotation2d;
import edu.wpi.first.wpilibj.geometry.Translation2d;
import edu.wpi.first.wpilibj.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.kinematics.SwerveDriveKinematics;
import edu.wpi.first.wpilibj.kinematics.SwerveDriveOdometry;
import edu.wpi.first.wpilibj.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInLayouts;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInWidgets;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardLayout;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.bionic.Conversion;
import frc.robot.Robot;

public abstract class AbstractDrivetrain extends SubsystemBase {
  public AbstractSwerveModule    swerveRF; // right front
  public AbstractSwerveModule    swerveLF; // left front
  public AbstractSwerveModule    swerveLR; // left rear
  public AbstractSwerveModule    swerveRR; // right rear
  public SwerveModuleState[]     swerveModuleStates = new SwerveModuleState[4];


  private SwerveDriveKinematics   kinematics;
  private SwerveDriveOdometry     odometry;
  private Pose2d                  currentPose;

  private double                  kMaxSpeed = Conversion.fpsToMps(18);  // meters per second

  //Shuffleboard Related
  private String                   name;
  private NetworkTableEntry        sb_Current_Pose_X;
  private NetworkTableEntry        sb_Current_Pose_Y;
  private NetworkTableEntry        sb_Current_Pose_Rotation;

  




  /**
   * Extending class must implement `getGyroAngle` method which returns the
   * robot's rotation in degrees. Degrees can be continuous, so result may be
   * outside range of [0.0, 360.0]. Degrees are measured *counterclockwise*
   * from zero, where zero is down-field towards opponent. Because degrees are
   * measured counterclockwise, the result may need to be negated, as devices
   * like NavX return degrees measured cockwise from zero.
   */
  abstract public double getGyroAngle();

  /**
   * Extending class must implement resetGyroAngle.
   * Mostly used for debugging
   */
  abstract public void resetGyroAngle();


  public void initialize (AbstractSwerveModule swerveRF,
                          AbstractSwerveModule swerveLF,
                          AbstractSwerveModule swerveLR,
                          AbstractSwerveModule swerveRR,
                          double kHalfWheelBaseWidthInches,
                          double kHalfWheelBaseLengthInches,
                          String name) {

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

    this.name = name;
    
    kHalfWheelBaseWidthMeters  = frc.bionic.Conversion.inchesToMeters(kHalfWheelBaseWidthInches);
    kHalfWheelBaseLengthMeters = frc.bionic.Conversion.inchesToMeters(kHalfWheelBaseLengthInches);

    frontRightLocation = new Translation2d(kHalfWheelBaseWidthMeters,  -kHalfWheelBaseLengthMeters);
    frontLeftLocation  = new Translation2d(kHalfWheelBaseWidthMeters,  kHalfWheelBaseLengthMeters);
    backLeftLocation   = new Translation2d(-kHalfWheelBaseWidthMeters, kHalfWheelBaseLengthMeters);
    backRightLocation  = new Translation2d(-kHalfWheelBaseWidthMeters, -kHalfWheelBaseLengthMeters);

    kinematics = new SwerveDriveKinematics(frontRightLocation, frontLeftLocation, backLeftLocation, backRightLocation);

    //Initilizes odometry with all 0 values (0,0,0)
    odometry = new SwerveDriveOdometry(
      kinematics,
      Rotation2d.fromDegrees(getGyroAngle()),
      new Pose2d(0.0, 0.0, Rotation2d.fromDegrees(0)));

    // initShuffleboard();
  }

  public double getMaxSpeed() {
    return kMaxSpeed;
  }

  public void setMaxSpeed(double maxSpeed) {
    kMaxSpeed = maxSpeed;
  }

  public void periodic() {
    //TODO: Uncomment the periodic calls
    swerveRF.periodic();
    swerveLF.periodic();
    swerveLR.periodic();
    swerveRR.periodic();

    // currentPose = odometry.update(Rotation2d.fromDegrees(getGyroAngle()),
    //                               swerveRF.getModuleState(), 
    //                               swerveLF.getModuleState(), 
    //                               swerveLR.getModuleState(), 
    //                               swerveRR.getModuleState());

    // syncShuffleboard();
  }

  /**
   * Method to drive the robot using joystick info.
   *
   * @param xSpeed Speed of the robot in the x direction (forward).
   * @param ySpeed Speed of the robot in the y direction (sideways).
   * @param rotate Angular rate of the robot.
   */
  public void drive(double xSpeed, double ySpeed, double rotate) {
    // Scale requested speed percentage [-1, 1]. to meters per second
    // throw new Error("Dont Call Me");
    xSpeed *= kMaxSpeed;
    ySpeed *= kMaxSpeed;
    rotate *= kMaxSpeed;

    // System.out.println("Drivetrain Drive Called");

    var chassisSpeeds = 
      ChassisSpeeds.fromFieldRelativeSpeeds(xSpeed, ySpeed, rotate, 
                                            Rotation2d.fromDegrees(this.getGyroAngle()));

    var swerveModuleStates = kinematics.toSwerveModuleStates(chassisSpeeds);
    SwerveDriveKinematics.normalizeWheelSpeeds(swerveModuleStates, kMaxSpeed);

    if (! Robot.debugDash.motorTab.useMotorControl()) {
      actuateModules(swerveModuleStates);
    }    
  }

  public void actuateModules(SwerveModuleState[] states){
    swerveRF.setModuleState(states[0]);
    swerveLF.setModuleState(states[1]);
    swerveLR.setModuleState(states[2]);
    swerveRR.setModuleState(states[3]);
  }

  /**
   * Orient the wheels such that the robot attempts to prevent being pushed
   * around by other robots.
   */
  // public void lockInPlace() {
  //   swerveRF.setModuleState(new SwerveModuleState(0, Rotation2d.fromDegrees(45)));
  //   swerveLF.setModuleState(new SwerveModuleState(0, Rotation2d.fromDegrees(-45)));
  //   swerveLR.setModuleState(new SwerveModuleState(0, Rotation2d.fromDegrees(45)));
  //   swerveRR.setModuleState(new SwerveModuleState(0, Rotation2d.fromDegrees(-45)));
  // }

  // public void resetOdometry(Pose2d resetPose){
  //   odometry.resetPosition(resetPose, resetPose.getRotation());
  // }

  //Acessor Methods
  // public SwerveDriveKinematics getKinematics(){
  //   return kinematics;
  // }

  // public Pose2d getCurrentPose(){
  //   return currentPose;
  // }

  /**
   * 
   * @return RF, LF, LR, RR
   */
  public SwerveModuleState[] getSwerveModuleStates(){
    swerveModuleStates[0] = swerveRF.getModuleState();
    swerveModuleStates[1] = swerveLF.getModuleState();
    swerveModuleStates[2] = swerveLR.getModuleState();
    swerveModuleStates[3] = swerveRR.getModuleState();

    return swerveModuleStates;
  }

  /**
   * Initialize the shuffleboard interface for this motor
  */
  // protected void initShuffleboard(){
  //   ShuffleboardTab           tab;
  //   ShuffleboardLayout        layout;

  //   tab              = Shuffleboard.getTab(name);
  //   layout           = tab.getLayout("Current Pose: " + name, BuiltInLayouts.kList);

  //   sb_Current_Pose_X = layout.add("X", 0).getEntry();
  //   sb_Current_Pose_Y = layout.add("Y", 0).getEntry();
  //   sb_Current_Pose_Rotation = layout.add("Rotation", 0).getEntry();
    
  // }

  /**
   * Update dynamic values on shuffleboard, and read values and reset based on
   * read values, any settable parameters.
  */
  // void syncShuffleboard(){
  //   // Sets the current pose X , Y, and Rotation to the shuffleboard variables to get updated
  //   sb_Current_Pose_X.setDouble(odometry.getPoseMeters().getX());
  //   sb_Current_Pose_Y.setDouble(odometry.getPoseMeters().getY());
  //   sb_Current_Pose_Rotation.setDouble(odometry.getPoseMeters().getRotation().getDegrees());
  // }


}
