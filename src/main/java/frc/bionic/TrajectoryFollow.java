package frc.bionic;

import java.util.List;

import edu.wpi.first.wpilibj.controller.PIDController;
import edu.wpi.first.wpilibj.controller.ProfiledPIDController;
import edu.wpi.first.wpilibj.geometry.Pose2d;
import edu.wpi.first.wpilibj.geometry.Translation2d;
import edu.wpi.first.wpilibj.geometry.Rotation2d;
import edu.wpi.first.wpilibj.trajectory.Trajectory;
import edu.wpi.first.wpilibj.trajectory.TrajectoryConfig;
import edu.wpi.first.wpilibj.trajectory.TrajectoryGenerator;
import edu.wpi.first.wpilibj.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.SwerveControllerCommand;
import frc.bionic.swerve.AbstractDrivetrain;

public class TrajectoryFollow {

  public SequentialCommandGroup getAutonomousCommand(AbstractDrivetrain m_robotDrive) {

    PIDController xController = new PIDController(1, 0, 0);
    PIDController yController = new PIDController(1, 0, 0);
    ProfiledPIDController thetaController = new ProfiledPIDController(1, 0, 0, new TrapezoidProfile.Constraints(6.28, 3.14));
  
    TrajectoryConfig config =
          new TrajectoryConfig(
                  2.0,
                  Conversion.inchesToMeters(144))
              // Add kinematics to ensure max speed is actually obeyed
              .setKinematics(m_robotDrive.getKinematics());
  
      // An example trajectory to follow.  All units in meters.
      Trajectory exampleTrajectory =
          TrajectoryGenerator.generateTrajectory(
              // Start at the origin facing the +X direction
              new Pose2d(0, 0, new Rotation2d(0)),
              // Pass through these two interior waypoints, making an 's' curve path
              List.of(new Translation2d(1, 1), new Translation2d(2, -1)),
              // End 3 meters straight ahead of where we started, facing forward
              new Pose2d(3, 0, new Rotation2d(0)),
              config);
              
      thetaController.enableContinuousInput(-Math.PI, Math.PI);
  
      SwerveControllerCommand swerveControllerCommand =
          new SwerveControllerCommand(
              exampleTrajectory,
              m_robotDrive::getCurrentPose, // Functional interface to feed supplier
              m_robotDrive.getKinematics(),
  
              // Position controllers
              xController,
              yController,
              thetaController,
              m_robotDrive::actuateModules,
              m_robotDrive);
  
      // Reset odometry to the starting pose of the trajectory.
      m_robotDrive.resetOdometry(exampleTrajectory.getInitialPose());
  
      // Run path following command, then stop at the end.
      return swerveControllerCommand.andThen(() -> m_robotDrive.drive(0.0, 0.0, 0.0));
  }
}
