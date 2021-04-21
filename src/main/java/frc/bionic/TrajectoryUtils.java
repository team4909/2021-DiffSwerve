package frc.bionic;

import java.io.File;
import java.io.FileWriter;
import java.io.IOException;
import java.util.Arrays;
import java.util.function.Consumer;
import java.util.function.Supplier;

import edu.wpi.first.wpilibj.controller.PIDController;
import edu.wpi.first.wpilibj.controller.ProfiledPIDController;
import edu.wpi.first.wpilibj.geometry.Pose2d;
import edu.wpi.first.wpilibj.geometry.Translation2d;
import edu.wpi.first.wpilibj.kinematics.SwerveDriveKinematics;
import edu.wpi.first.wpilibj.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.trajectory.Trajectory;
import edu.wpi.first.wpilibj.trajectory.TrajectoryConfig;
import edu.wpi.first.wpilibj.trajectory.TrajectoryGenerator;
import edu.wpi.first.wpilibj.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.SwerveControllerCommand;
import frc.bionic.swerve.AbstractDrivetrain;

public class TrajectoryUtils {
    
    public static void supply(Trajectory trajectory, AbstractDrivetrain drivetrain){
        Supplier<Pose2d> currentPose = () -> drivetrain.getCurrentPose();
        SwerveDriveKinematics kinematics = drivetrain.getKinematics();
        PIDController xController = new PIDController(1, 0, 0);
        PIDController yController = new PIDController(1, 0, 0);
        ProfiledPIDController thetaController = new ProfiledPIDController(1, 0, 0, new TrapezoidProfile.Constraints(6.28, 3.14));
        Consumer<SwerveModuleState[]> outputModuleStates = states -> {};
        outputModuleStates.accept(drivetrain.getSwerveModuleStates());
        
        new TrajectoryUtils().followTrajectory(trajectory, currentPose, kinematics, xController, yController, thetaController, outputModuleStates, drivetrain);
      }

      public Trajectory generateTrajectory(Pose2d startPos, Pose2d endPos, TrajectoryConfig config, Translation2d... interiorWaypoints) {
        Trajectory returnTrajectory = TrajectoryGenerator.generateTrajectory(startPos, Arrays.asList(interiorWaypoints), endPos, config);
        FileWriter fileWriter;
        try {
          fileWriter = new FileWriter(new File("TrajectoryOutput.txt"));
          fileWriter.write(returnTrajectory.toString());
        } catch (IOException e) {}
        return returnTrajectory;
      }

      public SequentialCommandGroup followTrajectory(Trajectory trajectory, Supplier<Pose2d> pose, SwerveDriveKinematics kinematics,
      PIDController xController, PIDController yController, ProfiledPIDController thetaController,
      Consumer<SwerveModuleState[]> outputModuleStates, AbstractDrivetrain drivetrain){
        //Creates a new SwerveControllerCommand object for controlling the robot
        SwerveControllerCommand swerveControllerCommand = new SwerveControllerCommand(trajectory, drivetrain::getCurrentPose, kinematics, xController, yController, thetaController, drivetrain::actuateModules, drivetrain);
        //Resets Initial Odometry to the first Pose in the Trajectory
        drivetrain.resetOdometry(trajectory.getInitialPose());    
        //Runs the trajectory and then stops the drivetrain
        return swerveControllerCommand.andThen(() -> drivetrain.drive(0, 0, 0));
      }


}
