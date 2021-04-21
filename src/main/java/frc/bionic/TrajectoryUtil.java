package frc.bionic;

import java.util.function.Consumer;
import java.util.function.Supplier;

import edu.wpi.first.wpilibj.controller.PIDController;
import edu.wpi.first.wpilibj.controller.ProfiledPIDController;
import edu.wpi.first.wpilibj.geometry.Pose2d;
import edu.wpi.first.wpilibj.kinematics.SwerveDriveKinematics;
import edu.wpi.first.wpilibj.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.trajectory.Trajectory;
import edu.wpi.first.wpilibj.trajectory.TrapezoidProfile;
import frc.bionic.swerve.AbstractDrivetrain;
import frc.bionic.swerve.command.TrajectoryFollow;

public class TrajectoryUtil {
    
    public static void supply(Trajectory trajectory, AbstractDrivetrain drivetrain){
        Supplier<Pose2d> currentPose = () -> drivetrain.getCurrentPose();
        SwerveDriveKinematics kinematics = drivetrain.getKinematics();
        PIDController xController = new PIDController(1, 0, 0);
        PIDController yController = new PIDController(1, 0, 0);
        ProfiledPIDController thetaController = new ProfiledPIDController(1, 0, 0, new TrapezoidProfile.Constraints(6.28, 3.14));
        Consumer<SwerveModuleState[]> outputModuleStates = states -> {};
        outputModuleStates.accept(drivetrain.getSwerveModuleStates());
        
        new TrajectoryFollow(trajectory, currentPose, kinematics, xController, yController, thetaController, outputModuleStates, drivetrain);
      }
}
