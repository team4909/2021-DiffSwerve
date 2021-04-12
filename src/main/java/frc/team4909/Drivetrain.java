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

import frc.bionic.swerve.AbstractDrivetrain;
import edu.wpi.first.wpilibj.SerialPort;
import edu.wpi.first.wpilibj.geometry.Rotation2d;
import edu.wpi.first.wpilibj.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import com.kauailabs.navx.frc.AHRS;

public class Drivetrain extends AbstractDrivetrain {
  public frc.team4909.SwerveModule swerveRF; // right front
  public frc.peyton.SwerveModule   swerveLF; // left front
  public frc.peyton.SwerveModule   swerveLR; // left rear
  public frc.peyton.SwerveModule   swerveRR; // right rear

  private AHRS navX;

  public Drivetrain() {
    double                kHalfWheelBaseWidthInches = 16.825108;
    double                kHalfWheelBaseLengthInches = 16.825108;

    swerveRF = new frc.team4909.SwerveModule(1, 2, 0,   10.4, "RF", "Peyton");
    swerveLF = new frc.peyton.SwerveModule(3, 4, 2,  -21.8, "LF", "Peyton");
    swerveLR = new frc.peyton.SwerveModule(5, 6, 4,    2.3, "LR", "Peyton");
    swerveRR = new frc.peyton.SwerveModule(7, 8, 6,  -18.9, "RR", "Peyton");


    this.initialize(swerveRF, swerveLF, swerveLR, swerveRR,
                    kHalfWheelBaseWidthInches, kHalfWheelBaseLengthInches);

    navX = new AHRS(SerialPort.Port.kMXP);
    navX.reset();
    SmartDashboard.putBoolean("NavX Reset", false);
  }

  // interface (Subsystem) implementation
  public void periodic() {
    super.periodic();

    SmartDashboard.putData("NavX", navX);
    SmartDashboard.putNumber("Gyro Angle", navX.getAngle());

    if (SmartDashboard.getBoolean("NavX Reset", false)) {
      SmartDashboard.putBoolean("NavX Reset", false);
      navX.reset();
    }
  }

  // abstract superclass implementation
  public double getGyroAngle(){
    // negate result because NavX returns degrees measured clockwise from zero,
    // whereas the defined interface that this method implements states that
    // the method must return degrees measured counterclockwise from zero.
    return -navX.getAngle();
  }

  public void lockInPlace() {
    swerveLF.setModuleState(new SwerveModuleState(0, Rotation2d.fromDegrees(-45)));
    swerveRF.setModuleState(new SwerveModuleState(0, Rotation2d.fromDegrees(45)));
    swerveLR.setModuleState(new SwerveModuleState(0, Rotation2d.fromDegrees(45)));
    swerveRR.setModuleState(new SwerveModuleState(0, Rotation2d.fromDegrees(-45)));
  }
}
