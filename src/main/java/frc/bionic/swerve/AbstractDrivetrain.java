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

public abstract class AbstractDrivetrain extends SubsystemBase {
  private AbstractSwerveModule    swerveRF; // right front
  private AbstractSwerveModule    swerveLF; // left front
  private AbstractSwerveModule    swerveLR; // left rear
  private AbstractSwerveModule    swerveRR; // right rear
  public SwerveModuleState[]      swerveModuleStates;


  private SwerveDriveKinematics   kinematics;
  private SwerveDriveOdometry     odometry;
  private Pose2d                  currentPose;

  private double                  kMaxSpeed = 2.0;  // meters per second

  //Shuffleboard Related
  private String                   name;
  private NetworkTableEntry        sb_Current_Pose_X;
  private NetworkTableEntry        sb_Current_Pose_Y;
  private NetworkTableEntry        sb_Current_Pose_Rotation;

  private NetworkTableEntry sb_modLF, sb_modRF, sb_modLR, sb_modRR;
  private NetworkTableEntry sb_motorA, sb_motorB;
  private NetworkTableEntry sb_sel_motor, sb_sel_module;




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

    initShuffleboard();
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

    currentPose = odometry.update(Rotation2d.fromDegrees(getGyroAngle()),
                                  swerveRF.getModuleState(), 
                                  swerveLF.getModuleState(), 
                                  swerveLR.getModuleState(), 
                                  swerveRR.getModuleState());

    syncShuffleboard();
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
    xSpeed *= kMaxSpeed;
    ySpeed *= kMaxSpeed;
    rotate *= kMaxSpeed;

    var chassisSpeeds = 
      ChassisSpeeds.fromFieldRelativeSpeeds(xSpeed, ySpeed, rotate, 
                                            Rotation2d.fromDegrees(this.getGyroAngle()));

    var swerveModuleStates = kinematics.toSwerveModuleStates(chassisSpeeds);
    SwerveDriveKinematics.normalizeWheelSpeeds(swerveModuleStates, kMaxSpeed);
    
    actuateModules(swerveModuleStates);
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
  public void lockInPlace() {
    swerveRF.setModuleState(new SwerveModuleState(0, Rotation2d.fromDegrees(45)));
    swerveLF.setModuleState(new SwerveModuleState(0, Rotation2d.fromDegrees(-45)));
    swerveLR.setModuleState(new SwerveModuleState(0, Rotation2d.fromDegrees(45)));
    swerveRR.setModuleState(new SwerveModuleState(0, Rotation2d.fromDegrees(-45)));
  }

  public void resetOdometry(Pose2d resetPose){
    odometry.resetPosition(resetPose, resetPose.getRotation());
  }

  //Acessor Methods
  public SwerveDriveKinematics getKinematics(){
    return kinematics;
  }

  public Pose2d getCurrentPose(){
    return currentPose;
  }

  /**
   * 
   * @return RF, LF, LR, RR
   */
  public SwerveModuleState[] getSwerveModuleStates(){
    return swerveModuleStates;
  }

  /**
   * Initialize the shuffleboard interface for this motor
  */
  protected void initShuffleboard(){
    ShuffleboardTab           tab;
    ShuffleboardLayout        layout;

    tab              = Shuffleboard.getTab(name);
    layout           = tab.getLayout("Current Pose: " + name, BuiltInLayouts.kList);

    sb_Current_Pose_X = layout.add("X", 0).getEntry();
    sb_Current_Pose_Y = layout.add("Y", 0).getEntry();
    sb_Current_Pose_Rotation = layout.add("Rotation", 0).getEntry();

    // var moduleTab = Shuffleboard.getTab("Module");
    int row = 0;
    // module selector
    // Yaw PID
    // Yaw RPM
    // Yaw Heading
    // Translation RPM

    var motorTab = Shuffleboard.getTab("Motor");
    row = 0;
    // Module selector
    {
      var moduleSelector = motorTab.getLayout("Module Selector", BuiltInLayouts.kGrid);
      moduleSelector.withProperties(Map.of("Number of columns", 2, "Number of rows", 2, "Label position", "HIDDEN"));
      moduleSelector.withSize(2, 2);
      moduleSelector.withPosition(0, row);

      sb_modLF = moduleSelector.add("LF", false).withSize(1, 1).withPosition(0, 0).withWidget(BuiltInWidgets.kToggleButton).getEntry();
      sb_modRF = moduleSelector.add("RF", false).withSize(1, 1).withPosition(1, 0).withWidget(BuiltInWidgets.kToggleButton).getEntry();
      sb_modLR = moduleSelector.add("LR", false).withSize(1, 1).withPosition(0, 1).withWidget(BuiltInWidgets.kToggleButton).getEntry();
      sb_modRR = moduleSelector.add("RR", false).withSize(1, 1).withPosition(1, 1).withWidget(BuiltInWidgets.kToggleButton).getEntry();
    }
    // Motor selector
    {
      var motorSelector = motorTab.getLayout("Motor Selector", BuiltInLayouts.kGrid);
      motorSelector.withProperties(Map.of("Number of columns", 2, "Number of rows", 1, "Label position", "HIDDEN"));
      motorSelector.withSize(2, 2);
      motorSelector.withPosition(2, row);

      sb_motorA = motorSelector.add("A", false).withSize(1, 1).withPosition(0, 0).withWidget(BuiltInWidgets.kToggleButton).getEntry();
      sb_motorB = motorSelector.add("B", false).withSize(1, 1).withPosition(1, 0).withWidget(BuiltInWidgets.kToggleButton).getEntry();
    }
    {
      var selected = motorTab.getLayout("Selected", BuiltInLayouts.kGrid);
      selected.withProperties(Map.of("Number of columns", 2, "Number of rows", 1));
      selected.withSize(2, 2);
      selected.withPosition(4, row);

      sb_sel_module  = selected.add("Mod", "").withSize(1, 1).withPosition(0, 0).getEntry();
      sb_sel_motor = selected.add("Motor", "").withSize(1, 1).withPosition(1, 0).getEntry();
    }
  }

  /**
   * Update dynamic values on shuffleboard, and read values and reset based on
   * read values, any settable parameters.
  */
  void syncShuffleboard(){
    // Sets the current pose X , Y, and Rotation to the shuffleboard variables to get updated
    sb_Current_Pose_X.setDouble(odometry.getPoseMeters().getX());
    sb_Current_Pose_Y.setDouble(odometry.getPoseMeters().getY());
    sb_Current_Pose_Rotation.setDouble(odometry.getPoseMeters().getRotation().getDegrees());

    // treat the selectors like radio buttons
    if (sb_motorA.getBoolean(false)) {
      sb_motorA.setBoolean(false);
      sb_sel_motor.setString("A");
    }
    if (sb_motorB.getBoolean(false)) {
      sb_motorB.setBoolean(false);
      sb_sel_motor.setString("B");
    }

    if (sb_modLF.getBoolean(false)) {
      sb_modLF.setBoolean(false);
      sb_sel_module.setString("LF");
    }
    if (sb_modRF.getBoolean(false)) {
      sb_modRF.setBoolean(false);
      sb_sel_module.setString("RF");
    }
    if (sb_modLR.getBoolean(false)) {
      sb_modLR.setBoolean(false);
      sb_sel_module.setString("LR");
    }
    if (sb_modRR.getBoolean(false)) {
      sb_modRR.setBoolean(false);
      sb_sel_module.setString("RR");
    }

  }

}
