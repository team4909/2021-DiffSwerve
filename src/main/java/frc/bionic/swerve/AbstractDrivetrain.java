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

    

    // Motor tab
    {
      int row = 0;
      var motorTab = Shuffleboard.getTab("Motor");
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

        sb_sel_module = selected.add("Mod", "").withSize(1, 1).withPosition(0, 0).getEntry();
        sb_sel_motor  = selected.add("Motor", "").withSize(1, 1).withPosition(1, 0).getEntry();
      }
      // setpoints
      {
        //0, 100, 500, 1000, 1500, 2000, text, apply
        var setpoints = motorTab.getLayout("Setpoints", BuiltInLayouts.kGrid);
        setpoints.withProperties(Map.of("Number of columns", 4, "Number of rows", 2, "Label position", "HIDDEN"));
        setpoints.withSize(6, 2);
        setpoints.withPosition(6, row);

        setpoints.add("0",    false).withSize(1, 1).withPosition(0, 0).withWidget(BuiltInWidgets.kToggleButton).getEntry();
        setpoints.add("100",  false).withSize(1, 1).withPosition(1, 0).withWidget(BuiltInWidgets.kToggleButton).getEntry();
        setpoints.add("500",  false).withSize(1, 1).withPosition(2, 0).withWidget(BuiltInWidgets.kToggleButton).getEntry();
        setpoints.add("1000", false).withSize(1, 1).withPosition(3, 0).withWidget(BuiltInWidgets.kToggleButton).getEntry();

        setpoints.add("1500",  false).withSize(1, 1).withPosition(0, 1).withWidget(BuiltInWidgets.kToggleButton).getEntry();
        setpoints.add("2000",  false).withSize(1, 1).withPosition(1, 1).withWidget(BuiltInWidgets.kToggleButton).getEntry();
        setpoints.add("input",  3000).withSize(1, 1).withPosition(2, 1).getEntry();
        setpoints.add("Apply", false).withSize(1, 1).withPosition(3, 1).withWidget(BuiltInWidgets.kToggleButton).getEntry();
      }
      row=2;
      // Graphs
      {
        motorTab.add("Motor RPM",    0).withSize(6, 5).withPosition(0, row).withWidget(BuiltInWidgets.kGraph).getEntry();
        motorTab.add("Motor Error",  0).withSize(6, 5).withPosition(6, row).withWidget(BuiltInWidgets.kGraph).getEntry();
      }

      // motor PID
      {
        //kP, kI, kIz, kD, kF, max, min, apply
        var motorPidOuter = motorTab.getLayout("Motor PID", BuiltInLayouts.kGrid);
        motorPidOuter.withProperties(Map.of("Number of columns", 1, "Number of rows", 2, "Label position", "HIDDEN"));
        motorPidOuter.withSize(3, 5);
        motorPidOuter.withPosition(12, row);

        motorPidOuter.add("Apply", false).withSize(1, 1).withPosition(0, 0).withWidget(BuiltInWidgets.kToggleButton).getEntry();

        var motorPid = motorPidOuter.getLayout("Inner", BuiltInLayouts.kGrid);
        motorPid.withProperties(Map.of("Number of columns", 2, "Number of rows", 4));
        motorPid.withPosition(0, 1);
        

        motorPid.add("kP",       0).withSize(1, 1).withPosition(0, 0).getEntry();
        motorPid.add("kI",       0).withSize(1, 1).withPosition(0, 1).getEntry();
        motorPid.add("kIz",      0).withSize(1, 1).withPosition(0, 2).getEntry();
        motorPid.add("kD",       0).withSize(1, 1).withPosition(0, 3).getEntry();

        motorPid.add("kF",        0).withSize(1, 1).withPosition(1, 0).getEntry();
        motorPid.add("max",       0).withSize(1, 1).withPosition(1, 1).getEntry();
        motorPid.add("min",       0).withSize(1, 1).withPosition(1, 2).getEntry();
      }
    }

    // Module Tab --------------------------------
    {
      int row = 0;
      var moduleTab = Shuffleboard.getTab("Module");
      // Module selector
      {
        var moduleSelector = moduleTab.getLayout("Module Selector", BuiltInLayouts.kGrid);
        moduleSelector.withProperties(Map.of("Number of columns", 2, "Number of rows", 2, "Label position", "HIDDEN"));
        moduleSelector.withSize(2, 2);
        moduleSelector.withPosition(0, row);

        sb_modLF = moduleSelector.add("LF", false).withSize(1, 1).withPosition(0, 0).withWidget(BuiltInWidgets.kToggleButton).getEntry();
        sb_modRF = moduleSelector.add("RF", false).withSize(1, 1).withPosition(1, 0).withWidget(BuiltInWidgets.kToggleButton).getEntry();
        sb_modLR = moduleSelector.add("LR", false).withSize(1, 1).withPosition(0, 1).withWidget(BuiltInWidgets.kToggleButton).getEntry();
        sb_modRR = moduleSelector.add("RR", false).withSize(1, 1).withPosition(1, 1).withWidget(BuiltInWidgets.kToggleButton).getEntry();
      }

      {
        moduleTab.add("Selected Module", "").withSize(2, 2).withPosition(2, row).getEntry();
      }

      // Yaw Setpoints
      {
        //0, 45, 90, 135, 180, -90, text, apply
        var setpoints = moduleTab.getLayout("Yaw Setpoints", BuiltInLayouts.kGrid);
        setpoints.withProperties(Map.of("Number of columns", 4, "Number of rows", 2, "Label position", "HIDDEN"));
        setpoints.withSize(6, 2);
        setpoints.withPosition(4, row);

        setpoints.add("0",   false).withSize(1, 1).withPosition(0, 0).withWidget(BuiltInWidgets.kToggleButton).getEntry();
        setpoints.add("45",  false).withSize(1, 1).withPosition(1, 0).withWidget(BuiltInWidgets.kToggleButton).getEntry();
        setpoints.add("90",  false).withSize(1, 1).withPosition(2, 0).withWidget(BuiltInWidgets.kToggleButton).getEntry();
        setpoints.add("135", false).withSize(1, 1).withPosition(3, 0).withWidget(BuiltInWidgets.kToggleButton).getEntry();

        setpoints.add("180",   false).withSize(1, 1).withPosition(0, 1).withWidget(BuiltInWidgets.kToggleButton).getEntry();
        setpoints.add("-90",   false).withSize(1, 1).withPosition(1, 1).withWidget(BuiltInWidgets.kToggleButton).getEntry();
        setpoints.add("input",  3000).withSize(1, 1).withPosition(2, 1).getEntry();
        setpoints.add("Apply", false).withSize(1, 1).withPosition(3, 1).withWidget(BuiltInWidgets.kToggleButton).getEntry();
      }

      // // Translation Setpoints
      // {
      //   //0, 100, 500, 1000, 1500, 2000, text, apply
      //   var setpoints = moduleTab.getLayout("Translation Setpoints", BuiltInLayouts.kGrid);
      //   setpoints.withProperties(Map.of("Number of columns", 4, "Number of rows", 2, "Label position", "HIDDEN"));
      //   setpoints.withSize(6, 2);
      //   setpoints.withPosition(10, row);

      //   setpoints.add("0",    false).withSize(1, 1).withPosition(0, 0).withWidget(BuiltInWidgets.kToggleButton).getEntry();
      //   setpoints.add("100",  false).withSize(1, 1).withPosition(1, 0).withWidget(BuiltInWidgets.kToggleButton).getEntry();
      //   setpoints.add("500",  false).withSize(1, 1).withPosition(2, 0).withWidget(BuiltInWidgets.kToggleButton).getEntry();
      //   setpoints.add("1000", false).withSize(1, 1).withPosition(3, 0).withWidget(BuiltInWidgets.kToggleButton).getEntry();

      //   setpoints.add("1500",  false).withSize(1, 1).withPosition(0, 1).withWidget(BuiltInWidgets.kToggleButton).getEntry();
      //   setpoints.add("2000",  false).withSize(1, 1).withPosition(1, 1).withWidget(BuiltInWidgets.kToggleButton).getEntry();
      //   setpoints.add("input",  3000).withSize(1, 1).withPosition(2, 1).getEntry();
      //   setpoints.add("Apply", false).withSize(1, 1).withPosition(3, 1).withWidget(BuiltInWidgets.kToggleButton).getEntry();
      // }

      row = 2;
      //graphs
      {
        moduleTab.add("Yaw Heading",     0).withSize(6, 5).withPosition(0,  row).withWidget(BuiltInWidgets.kGraph).getEntry();
        moduleTab.add("Yaw Error",       0).withSize(6, 5).withPosition(6,  row).withWidget(BuiltInWidgets.kGraph).getEntry();
        // moduleTab.add("Translation RPM", 0).withSize(5, 5).withPosition(12, row).withWidget(BuiltInWidgets.kGraph).getEntry();
      }
      
      // Yaw PID
      {
        //kP, kI, kIz, kD, kF, max, min, apply
        var motorPidOuter = moduleTab.getLayout("Yaw PID", BuiltInLayouts.kGrid);
        motorPidOuter.withProperties(Map.of("Number of columns", 1, "Number of rows", 2, "Label position", "HIDDEN"));
        motorPidOuter.withSize(3, 5);
        motorPidOuter.withPosition(12, row);

        motorPidOuter.add("Apply", false).withSize(1, 1).withPosition(0, 0).withWidget(BuiltInWidgets.kToggleButton).getEntry();

        var motorPid = motorPidOuter.getLayout("Inner", BuiltInLayouts.kGrid);
        motorPid.withProperties(Map.of("Number of columns", 2, "Number of rows", 4));
        motorPid.withPosition(0, 1);
        

        motorPid.add("kP",       0).withSize(1, 1).withPosition(0, 0).getEntry();
        motorPid.add("kI",       0).withSize(1, 1).withPosition(0, 1).getEntry();
        motorPid.add("kIz",      0).withSize(1, 1).withPosition(0, 2).getEntry();
        motorPid.add("kD",       0).withSize(1, 1).withPosition(0, 3).getEntry();

        motorPid.add("kF",        0).withSize(1, 1).withPosition(1, 0).getEntry();
        motorPid.add("max",       0).withSize(1, 1).withPosition(1, 1).getEntry();
        motorPid.add("min",       0).withSize(1, 1).withPosition(1, 2).getEntry();
      }


    // Yaw PID
    // Yaw RPM
    // Yaw Heading
    // Translation RPM

 
      // // Yaw
      // {
      //   var yaw = moduleTab.getLayout("Yaw", BuiltInLayouts.kGrid);
      //   yaw.withProperties(Map.of("Number of columns", 2, "Number of rows", 1));
      //   yaw.withSize(2, 2);
      //   yaw.withPosition(5, row);

      //   yaw.add("RPM",      0).withSize(1, 1).withPosition(0, 0).getEntry();
      //   yaw.add("Heading",  0).withSize(1, 1).withPosition(1, 0).getEntry();
      // }
      
      // Translation
      {

      }
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
