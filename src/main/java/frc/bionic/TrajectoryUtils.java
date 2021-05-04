package frc.bionic;

import java.util.Arrays;
import java.util.List;

import edu.wpi.first.wpilibj.geometry.Pose2d;
import edu.wpi.first.wpilibj.geometry.Rotation2d;
import edu.wpi.first.wpilibj.geometry.Translation2d;
import edu.wpi.first.wpilibj.trajectory.Trajectory;
import edu.wpi.first.wpilibj.trajectory.TrajectoryConfig;
import edu.wpi.first.wpilibj.trajectory.TrajectoryGenerator;
import frc.bionic.swerve.AbstractDrivetrain;

public class TrajectoryUtils {

      /**
       * 
       * @param startPos The starting position of the robot on the trajectory
       * @param endPos The ending position og the robot on the trajectory
       * @param config The Trajectory config object that defines max velocity and max acceleration
       * @param interiorWaypoints An Arraylist of Translation 2d object which generate the path in between the start and end
       * @return The generated trajectory based off of the given parameters
       */
      public static Trajectory generateTrajectory(Pose2d startPos, 
                                            double endX,
                                            double endY,
                                            double rotation, 
                                            AbstractDrivetrain drivetrain, 
                                            Translation2d... interiorWaypoints) {
            // Creates a new Trajectory Config, which stores Max Velocity, Max Acceleration and Kinematics
            TrajectoryConfig config =
                  new TrajectoryConfig(
                        // Max Velocity Meters Per Seconds
                        Conversion.inchesToMeters(120),
                        // Max Acceleration Meters Per Seconds Squared
                        Conversion.inchesToMeters(144))
                  // Add kinematics to ensure max speed is actually obeyed
                  .setKinematics(drivetrain.getKinematics());

            Pose2d endPos = new Pose2d(endX, endY, Rotation2d.fromDegrees(rotation));
            Trajectory returnTrajectory = TrajectoryGenerator.generateTrajectory(startPos, 
                                                                              Arrays.asList(interiorWaypoints), 
                                                                              endPos, 
                                                                              config);
            return returnTrajectory;
      }

      public static Trajectory generateSCurve(TrajectoryConfig config){
            Trajectory trajectory =
                  TrajectoryGenerator.generateTrajectory(
                  // Start at the origin facing the +X direction
                  new Pose2d(0, 0, new Rotation2d(0)),
                  // Pass through these two interior waypoints, making an 's' curve path
                  List.of(new Translation2d(1, 1), new Translation2d(2, -1)),
                  // End 3 meters straight ahead of where we started, facing forward
                  new Pose2d(3, 0, new Rotation2d(0)),
                  config);
                  
                  return trajectory;
      }
}