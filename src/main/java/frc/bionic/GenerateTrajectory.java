package frc.bionic;

import java.util.Arrays;
import java.util.List;

import edu.wpi.first.wpilibj.geometry.Pose2d;
import edu.wpi.first.wpilibj.geometry.Translation2d;
import edu.wpi.first.wpilibj.trajectory.Trajectory;
import edu.wpi.first.wpilibj.trajectory.TrajectoryConfig;
import edu.wpi.first.wpilibj.trajectory.TrajectoryGenerator;

public class GenerateTrajectory {

    Pose2d startPos;
    Pose2d endPos;
    TrajectoryConfig config;
    List<Translation2d> interiorWaypoints;
    
    public GenerateTrajectory(Pose2d startPos, Pose2d endPos, TrajectoryConfig config, Translation2d... interiorWaypoints) {
        this.startPos = startPos;
        this.endPos = endPos;
        this.config = config;
        this.interiorWaypoints = Arrays.asList(interiorWaypoints);
    }

    public Trajectory returnTrajectory(){
        Trajectory returnTrajectory = TrajectoryGenerator.generateTrajectory(startPos, interiorWaypoints, endPos, config);
        return returnTrajectory;
    }
}
