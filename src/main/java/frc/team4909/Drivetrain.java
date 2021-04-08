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

package frc.team4909;

import edu.wpi.first.wpilibj.SerialPort;
import edu.wpi.first.wpilibj.geometry.Rotation2d;
import edu.wpi.first.wpilibj.geometry.Translation2d;
import edu.wpi.first.wpilibj.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.kinematics.SwerveDriveKinematics;
import edu.wpi.first.wpilibj.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Subsystem;
import frc.bionic.swerve.IDrivetrainSubsystem;

import java.util.Arrays;
import java.util.List;
import java.util.stream.Collectors;

import com.kauailabs.navx.frc.*;

public class Drivetrain implements Subsystem, IDrivetrainSubsystem {
  public team4909SwerveModule swerveRF; // right front
  public team4909SwerveModule swerveLF; // left front
  public team4909SwerveModule swerveLR; // left rear
  public team4909SwerveModule swerveRR; // right rear

  private final double kHalfWheelBaseWidthInches;
  private final double kHalfWheelBaseWidthMeters;
  private final double kHalfWheelBaseLengthInches;
  private final double kHalfWheelBaseLengthMeters;

  private final double kMaxSpeed;

  private final Translation2d m_frontLeftLocation;
  private final Translation2d m_frontRightLocation;
  private final Translation2d m_backLeftLocation;
  private final Translation2d m_backRightLocation;

  private final SwerveDriveKinematics m_kinematics;
  private AHRS navX;

  public Drivetrain() {
    //TODO Test Values
    kHalfWheelBaseWidthInches = 0;
    kHalfWheelBaseWidthMeters = 0.0;

    kHalfWheelBaseLengthInches = 0.0;
    kHalfWheelBaseLengthMeters = 0.0;

    kMaxSpeed = 0.0; // x meters per second

    m_frontLeftLocation  = new Translation2d(kHalfWheelBaseWidthMeters, kHalfWheelBaseLengthMeters);
    m_frontRightLocation = new Translation2d(kHalfWheelBaseWidthMeters, -kHalfWheelBaseLengthMeters);
    m_backLeftLocation   = new Translation2d(-kHalfWheelBaseWidthMeters, kHalfWheelBaseLengthMeters);
    m_backRightLocation  = new Translation2d(-kHalfWheelBaseWidthMeters, -kHalfWheelBaseLengthMeters);

    m_kinematics = new SwerveDriveKinematics(m_frontLeftLocation, m_frontRightLocation,
                                             m_backLeftLocation, m_backRightLocation);

    swerveRF = new team4909SwerveModule(1, 2, 0, 1, "RF", "Bionic");
    swerveLF = new team4909SwerveModule(3, 4, 2, 3, "LF", "Bionic");
    swerveLR = new team4909SwerveModule(5, 6, 4, 5, "LR", "Bionic");
    swerveRR = new team4909SwerveModule(7, 8, 6, 7, "RR", "Bionic");

    navX = new AHRS(SerialPort.Port.kMXP);
    navX.reset();
    SmartDashboard.putBoolean("NavX Reset", false);
  }

  // interface (Subsystem) implementation
  public void periodic() {
    swerveRF.periodic();
    swerveLF.periodic();
    swerveLR.periodic();
    swerveRR.periodic();
    SmartDashboard.putData("NavX", navX);
    SmartDashboard.putNumber("Gyro Angle", navX.getAngle());

    if (SmartDashboard.getBoolean("NavX Reset", false)) {
      SmartDashboard.putBoolean("NavX Reset", false);
      navX.reset();
    }

  }

  // interface (IDrivetrainSubsystem) implementation
  public void drive(double xSpeed, double ySpeed, double rot) {
    var robotAngle = Rotation2d.fromDegrees(-navX.getAngle());
    var a = ChassisSpeeds.fromFieldRelativeSpeeds(xSpeed, ySpeed, rot, robotAngle);

    SmartDashboard.putString("ChassisSpeeds", a.toString());

    var swerveModuleStates = m_kinematics.toSwerveModuleStates(a);

    List<String> thing1 = Arrays.asList(swerveModuleStates).stream().map(i -> i.toString()).collect(Collectors.toList());
    SmartDashboard.putString("BeforeNormalize", thing1.toString());

    SwerveDriveKinematics.normalizeWheelSpeeds(swerveModuleStates, kMaxSpeed);

    List<String> thing2 = Arrays.asList(swerveModuleStates).stream().map(i -> i.toString()).collect(Collectors.toList());
    SmartDashboard.putString("AfterNormalize", thing2.toString());
    
    swerveLF.setModuleState(swerveModuleStates[0]);
    swerveRF.setModuleState(swerveModuleStates[1]);
    swerveLR.setModuleState(swerveModuleStates[2]);
    swerveRR.setModuleState(swerveModuleStates[3]);
  }

  // interface (IDrivetrainSubsystem) implementation
  public void lockInPlace() {
    swerveLF.setModuleState(new SwerveModuleState(0, Rotation2d.fromDegrees(-45)));
    swerveRF.setModuleState(new SwerveModuleState(0, Rotation2d.fromDegrees(45)));
    swerveLR.setModuleState(new SwerveModuleState(0, Rotation2d.fromDegrees(45)));
    swerveRR.setModuleState(new SwerveModuleState(0, Rotation2d.fromDegrees(-45)));
  }
}
