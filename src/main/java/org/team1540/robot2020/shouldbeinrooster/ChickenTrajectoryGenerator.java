package org.team1540.robot2020.shouldbeinrooster;

import edu.wpi.first.wpilibj.geometry.Pose2d;
import edu.wpi.first.wpilibj.geometry.Rotation2d;
import edu.wpi.first.wpilibj.geometry.Translation2d;
import edu.wpi.first.wpilibj.trajectory.Trajectory;
import edu.wpi.first.wpilibj.trajectory.TrajectoryConfig;
import edu.wpi.first.wpilibj.trajectory.TrajectoryGenerator;

import java.util.List;

public class ChickenTrajectoryGenerator {
    private static Pose2d lastPose = new Pose2d(0, 0, new Rotation2d(0));

    public static Trajectory generateTrajectory(List<Translation2d> waypoints, Pose2d end, TrajectoryConfig config) {
        Trajectory trajectory = TrajectoryGenerator.generateTrajectory(
                lastPose,
                waypoints,
                end,
                config
        );
        lastPose = end;
        return trajectory;
    }
}
