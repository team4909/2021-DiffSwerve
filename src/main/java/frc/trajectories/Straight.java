package frc.trajectories;

import com.kauailabs.navx.frc.AHRS;

import edu.wpi.first.wpilibj.geometry.Pose2d;
import edu.wpi.first.wpilibj.geometry.Rotation2d;
import frc.bionic.Conversion;
import frc.bionic.UserInterfaceElement;
import frc.peyton.Drivetrain;
import frc.robot.UserInterface;

public class Straight {

  Drivetrain drivetrain;

  public Straight(Drivetrain d){
    this.drivetrain = d;
  }

  public void generateTrajectory(){

    var staringPos = new Pose2d(Conversion.inchesToMeters(0), Conversion.inchesToMeters(0), Rotation2d.fromDegrees(drivetrain.getGyroAngle()));
    var endingPos = new Pose2d(Conversion.inchesToMeters(0), Conversion.inchesToMeters(36), Rotation2d.fromDegrees(drivetrain.getGyroAngle()));

    

  }
}