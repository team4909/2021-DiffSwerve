package frc.bionic;

import java.util.List;

import edu.wpi.first.wpilibj.controller.PIDController;
import edu.wpi.first.wpilibj.controller.ProfiledPIDController;
import edu.wpi.first.wpilibj.geometry.Pose2d;
import edu.wpi.first.wpilibj.geometry.Rotation2d;
import edu.wpi.first.wpilibj.geometry.Translation2d;
import edu.wpi.first.wpilibj.trajectory.Trajectory;
import edu.wpi.first.wpilibj.trajectory.TrajectoryConfig;
import edu.wpi.first.wpilibj.trajectory.TrajectoryGenerator;
import edu.wpi.first.wpilibj.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.SwerveControllerCommand;
import frc.bionic.swerve.AbstractDrivetrain;

public class TrajectoryFollow {

  public SequentialCommandGroup getAutonomousCommand(AbstractDrivetrain drivetrain) {

    // Creates a new PID Controller to control the X position of the Robot
    PIDController xController = new PIDController(1, 0, 0);
    // Creates a new PID Controller to control the Y Position of the Robot
    PIDController yController = new PIDController(1, 0, 0);
    // Creates a new PID Controller to control the angle of the robot, with Max Velocity and Max Acceleration constraints
    ProfiledPIDController thetaController = new ProfiledPIDController(1, 0, 0, new TrapezoidProfile.Constraints(6.28, 3.14));
    
    // Creates a new Trajectory Config, which stores Max Velocity, Max Acceleration and Kinematics
    TrajectoryConfig config =
    new TrajectoryConfig(
            // Max Velocity Meters Per Seconds
            Conversion.inchesToMeters(144),
            // Max Acceleration Meters Per Seconds Squared
            Conversion.inchesToMeters(144))
        // Add kinematics to ensure max speed is actually obeyed
        .setKinematics(drivetrain.getKinematics());

      // // Creates a new Trajectory | ALL UNITS ARE IN METERS
      Trajectory trajectory =
          TrajectoryGenerator.generateTrajectory(
              // Start at the origin facing the +X direction
            new Pose2d(0, 0, new Rotation2d(0)),
            // Pass through these two interior waypoints, making an 's' curve path
            List.of(new Translation2d(1, 1), new Translation2d(2, -1)),
            // End 3 meters straight ahead of where we started, facing forward
            new Pose2d(3, 0, new Rotation2d(0)),
              config);

      // Makes sure that the PID outputs values from -180 to 180 degrees
      thetaController.enableContinuousInput(-Math.PI, Math.PI);
  
      // Creates a new SwerveControllerCommand
      SwerveControllerCommand swerveControllerCommand =
          new SwerveControllerCommand(
              // The trajectory to follow
              trajectory,
              // A method refrence for constantly getting current position of the robot
              drivetrain::getCurrentPose,
              // Getting the kinematics from the drivetrain
              drivetrain.getKinematics(),
  
              // Position controllers
              xController,
              yController,
              thetaController,
              // A method refrence for setting the state of the modules
              drivetrain::actuateModules,
              // Requirment of a drivetrain subsystem
              drivetrain);
  
      // Reset odometry to the starting pose of the trajectory.
      drivetrain.resetOdometry(trajectory.getInitialPose());
  
      // Run path following command, then stop at the end.
      return swerveControllerCommand.andThen(() -> drivetrain.drive(0.0, 0.0, 0.0));
  }
}
