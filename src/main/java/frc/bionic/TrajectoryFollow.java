package frc.bionic;

import java.io.IOException;
import java.nio.file.Path;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj.controller.PIDController;
import edu.wpi.first.wpilibj.controller.ProfiledPIDController;
import edu.wpi.first.wpilibj.trajectory.Trajectory;
import edu.wpi.first.wpilibj.trajectory.TrajectoryUtil;
import edu.wpi.first.wpilibj.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.SwerveControllerCommand;
import frc.bionic.swerve.AbstractDrivetrain;

public class TrajectoryFollow {

  public SequentialCommandGroup getTrajectoryCommand(AbstractDrivetrain drivetrain, String trajectoryJSON) {

    // Creates a new PID Controller to control the X position of the Robot
    PIDController xController = new PIDController(3, 0, 0);
    // Creates a new PID Controller to control the Y Position of the Robot
    PIDController yController = new PIDController(3, 0, 0);
    // Creates a new PID Controller to control the angle of the robot, with Max Velocity and Max Acceleration constraints
    ProfiledPIDController thetaController = new ProfiledPIDController(1, 0, 0, new TrapezoidProfile.Constraints(6.28, 3.14));
    // Makes sure that the PID outputs values from -180 to 180 degrees
    thetaController.enableContinuousInput(-Math.PI, Math.PI);

    Trajectory trajectory = new Trajectory();
    Trajectory t = new Trajectory();
    
    try {
        Path trajectoryPath = Filesystem.getDeployDirectory().toPath().resolve(trajectoryJSON);
        trajectory = TrajectoryUtil.fromPathweaverJson(trajectoryPath);
    } catch (IOException ex) {
        DriverStation.reportError("Unable to open trajectory: " + trajectoryJSON, ex.getStackTrace());
    }

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

    // Reset odometry to the starting pose of the trajectory. This effectively transforms the trajectory to the current pose of the robot
    drivetrain.resetOdometry(trajectory.getInitialPose());
  
    // Run path following command, then stop at the end.
    return swerveControllerCommand.andThen(() -> drivetrain.drive(0.0, 0.0, 0.0));
  }
}
