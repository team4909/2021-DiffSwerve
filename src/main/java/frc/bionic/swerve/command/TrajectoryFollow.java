package frc.bionic.swerve.command;

import java.util.function.Consumer;
import java.util.function.Supplier;

import edu.wpi.first.wpilibj.controller.PIDController;
import edu.wpi.first.wpilibj.controller.ProfiledPIDController;
import edu.wpi.first.wpilibj.geometry.Pose2d;
import edu.wpi.first.wpilibj.kinematics.SwerveDriveKinematics;
import edu.wpi.first.wpilibj.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.trajectory.Trajectory;
import edu.wpi.first.wpilibj.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj2.command.Subsystem;
import edu.wpi.first.wpilibj2.command.SwerveControllerCommand;
import frc.peyton.Drivetrain;

public class TrajectoryFollow extends SwerveControllerCommand {

  Trajectory trajectory;
  Supplier<Pose2d> pose;
  SwerveDriveKinematics kinematics;
  PIDController xController; 
  PIDController yController; 
  ProfiledPIDController thetaController;
  Consumer<SwerveModuleState[]> outputModuleStates;
  Drivetrain drivetrain;



  public TrajectoryFollow(Trajectory trajectory, Supplier<Pose2d> pose, SwerveDriveKinematics kinematics,
      PIDController xController, PIDController yController, ProfiledPIDController thetaController,
      Consumer<SwerveModuleState[]> outputModuleStates, Drivetrain drivetrain)
  {
    super(trajectory, pose, kinematics, xController, yController, thetaController, outputModuleStates, drivetrain);

    this.trajectory = trajectory; 
    this.pose = pose; 
    this.kinematics = kinematics; 
    this.xController = xController;  
    this.yController = yController;  
    this.thetaController = thetaController; 
    this.outputModuleStates = outputModuleStates; 
    this.drivetrain = drivetrain;
  }

  public static void supply(Trajectory trajectory, Drivetrain drivetrain){
    Supplier<Pose2d> currentPose = () -> drivetrain.getCurrentPose();
    SwerveDriveKinematics kinematics = drivetrain.getKinematics();
    PIDController xController = new PIDController(1, 0, 0);
    PIDController yController = new PIDController(1, 0, 0);
    ProfiledPIDController thetaController = new ProfiledPIDController(1, 0, 0, TrapezoidProfile.Constraints(6.28, 3.14));
    Consumer<SwerveModuleState[]> = 



    new TrajectoryFollow(trajectory, pose, kinematics, xController, yController, thetaController, outputModuleStates, drivetrain)
  }

}
