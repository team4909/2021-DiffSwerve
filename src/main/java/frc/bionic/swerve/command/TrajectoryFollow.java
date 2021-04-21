package frc.bionic.swerve.command;

import java.util.function.Consumer;
import java.util.function.Supplier;

import edu.wpi.first.wpilibj.controller.PIDController;
import edu.wpi.first.wpilibj.controller.ProfiledPIDController;
import edu.wpi.first.wpilibj.geometry.Pose2d;
import edu.wpi.first.wpilibj.kinematics.SwerveDriveKinematics;
import edu.wpi.first.wpilibj.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.trajectory.Trajectory;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.SwerveControllerCommand;
import frc.bionic.swerve.AbstractDrivetrain;

public class TrajectoryFollow {

  Trajectory trajectory;
  Supplier<Pose2d> pose;
  SwerveDriveKinematics kinematics;
  PIDController xController; 
  PIDController yController; 
  ProfiledPIDController thetaController;
  Consumer<SwerveModuleState[]> outputModuleStates;
  AbstractDrivetrain drivetrain;


  //This class is meant to only be called from the Supply method in TrajectoryUtil.java
  public TrajectoryFollow(Trajectory trajectory, Supplier<Pose2d> pose, SwerveDriveKinematics kinematics,
      PIDController xController, PIDController yController, ProfiledPIDController thetaController,
      Consumer<SwerveModuleState[]> outputModuleStates, AbstractDrivetrain drivetrain)
  {
    //Sets arguments to class member variables to be used later
    this.trajectory = trajectory; 
    this.pose = pose; 
    this.kinematics = kinematics; 
    this.xController = xController;  
    this.yController = yController;  
    this.thetaController = thetaController; 
    this.outputModuleStates = outputModuleStates; 
    this.drivetrain = drivetrain;

    //Calls the method for following the trajectory
    followTrajectory();
  }

  public SequentialCommandGroup followTrajectory(){
    //Creates a new SwerveControllerCommand object for controlling the robot
    SwerveControllerCommand swerveControllerCommand = new SwerveControllerCommand(trajectory, drivetrain::getCurrentPose, kinematics, xController, yController, thetaController, drivetrain::actuateModules, drivetrain);
    //Resets Initial Odometry to the first Pose in the Trajectory
    drivetrain.resetOdometry(trajectory.getInitialPose());    
    //Runs the trajectory and then stops the drivetrain
    return swerveControllerCommand.andThen(() -> drivetrain.drive(0, 0, 0));
  }



}
