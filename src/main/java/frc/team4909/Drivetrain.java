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
import frc.bionic.swerve.Vision;
import edu.wpi.first.wpilibj.SerialPort;
import edu.wpi.first.wpilibj.geometry.Pose2d;
import edu.wpi.first.wpilibj.geometry.Rotation2d;
import edu.wpi.first.wpilibj.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import com.kauailabs.navx.frc.AHRS;

public class Drivetrain extends AbstractDrivetrain {
    public SwerveModule swerveRF; // right front
    public SwerveModule swerveLF; // left front
    public SwerveModule swerveLR; // left rear
    public SwerveModule swerveRR; // right rear

    private AHRS navX;

    public Drivetrain() {
        final String name = "Team4909";
        double kHalfWheelBaseWidthInches = 16.825108;
        double kHalfWheelBaseLengthInches = 16.825108;

        swerveRF = new SwerveModule(1, 2, 0, "RF", name);
        swerveLF = new SwerveModule(3, 4, 2, "LF", name);
        swerveLR = new SwerveModule(5, 6, 4, "LR", name);
        swerveRR = new SwerveModule(7, 8, 6, "RR", name);

        navX = new AHRS(SerialPort.Port.kMXP);
        navX.reset();
        // SmartDashboard.putBoolean("NavX Reset", false);

        this.initialize(swerveRF, swerveLF, swerveLR, swerveRR, kHalfWheelBaseWidthInches, kHalfWheelBaseLengthInches,
                name);
    }

    // interface (Subsystem) implementation
    public void periodic() {
        super.periodic();

    }

    // abstract superclass implementation
    public double getGyroAngle() {
        // negate result because NavX returns degrees measured clockwise from zero,
        // whereas the defined interface that this method implements states that
        // the method must return degrees measured counterclockwise from zero.
        return -navX.getAngle();
    }

    // abstract superclass implementation
    public void resetGyroAngle() {
        navX.reset();
    }

    // Infinite Recharge Game specific
    // public Pose2d getAbsolutePosition(){
    // //Distance to the powerport from the robot
    // double hypoteneuse =
    // new Vision(-56.4, 109.95, 29, 5, 0).calculateDistanceFromCameraHeight(0,0,0);

    // //Distance from the powerport to the initiation line
    // double base1 = 120;

    // //Distance from the robot to the interseciton of the powerport and the
    // initiaiton line
    // double base2 = Math.pow(hypoteneuse, 2) - Math.pow(base1, 2);

    // //Distace from the robot to the edge closest to the powerport
    // double position = base2 + 10; //TODO make the value added be right (check
    // CAD)

    // //Sets the position in relationto the side of the field closes to the
    // powerport
    // Pose2d absolutePosition =
    // getInversePose(new Pose2d(120, position,
    // this.getCurrentPose().getRotation()));

    // return absolutePosition;
    // }

    // // Might have to do this the other way around, needs more thought /
    // discussion`
    // public Pose2d getInversePose(Pose2d targetPose){
    // //Target Position - Current Position gives the position in relation to target
    // posiiton
    // Pose2d returnPose = new Pose2d(targetPose.getX() -
    // this.getCurrentPose().getX(),
    // targetPose.getY() - this.getCurrentPose().getY(),
    // new Rotation2d(targetPose.getRotation().getDegrees() -
    // this.getCurrentPose().getRotation().getDegrees()));
    // return returnPose;
    // }
}
