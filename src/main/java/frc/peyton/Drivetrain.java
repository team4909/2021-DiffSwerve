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

package frc.peyton;

import edu.wpi.first.wpilibj.geometry.Translation2d;
import edu.wpi.first.wpilibj.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.kinematics.SwerveDriveKinematics;

public class Drivetrain
{
  public PeytonSwerveModule    swerveRF; // right front
  public PeytonSwerveModule    swerveLF; // left front
  public PeytonSwerveModule    swerveLR; // left rear
  public PeytonSwerveModule    swerveRR; // right rear

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


  public Drivetrain()
  {
    kHalfWheelBaseWidthInches  = 15.0;
    kHalfWheelBaseWidthMeters  = frc.bionic.Conversion.inchesToMeters(kHalfWheelBaseWidthInches);

    kHalfWheelBaseLengthInches = 15.0;
    kHalfWheelBaseLengthMeters = frc.bionic.Conversion.inchesToMeters(kHalfWheelBaseLengthInches);
  
    kMaxSpeed = 3.0; // 3 meters per second
  
    m_frontLeftLocation  =
      new Translation2d( kHalfWheelBaseWidthMeters,  kHalfWheelBaseLengthMeters);
    m_frontRightLocation = 
      new Translation2d( kHalfWheelBaseWidthMeters, -kHalfWheelBaseLengthMeters);
    m_backLeftLocation   = 
      new Translation2d(-kHalfWheelBaseWidthMeters,  kHalfWheelBaseLengthMeters);
    m_backRightLocation  = 
      new Translation2d(-kHalfWheelBaseWidthMeters, -kHalfWheelBaseLengthMeters);
  
    m_kinematics = new SwerveDriveKinematics(m_frontLeftLocation, m_frontRightLocation,
                                             m_backLeftLocation, m_backRightLocation);

    swerveRF = new PeytonSwerveModule(1, 2, 0, 1, "RF", "Peyton");
    swerveLF = new PeytonSwerveModule(3, 4, 2, 3, "LF", "Peyton");
    swerveLR = new PeytonSwerveModule(5, 6, 4, 5, "LR", "Peyton");
    swerveRR = new PeytonSwerveModule(7, 8, 6, 7, "RR", "Peyton");
  }

  public void periodic()
  {
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
   * @param rot Angular rate of the robot.
   */
  public void drive(double xSpeed, double ySpeed, double rot)
  {
    var swerveModuleStates = m_kinematics.toSwerveModuleStates(new ChassisSpeeds(xSpeed, ySpeed, rot));
    SwerveDriveKinematics.normalizeWheelSpeeds(swerveModuleStates, kMaxSpeed);
    
    swerveLF.setModuleState(swerveModuleStates[0]);
    swerveRF.setModuleState(swerveModuleStates[1]);
    swerveLR.setModuleState(swerveModuleStates[2]);
    swerveRR.setModuleState(swerveModuleStates[3]);
  }
}
