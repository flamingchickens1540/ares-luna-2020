package org.team1540.robot2020.utils;

import edu.wpi.first.wpilibj.geometry.Pose2d;
import edu.wpi.first.wpilibj.geometry.Rotation2d;
import edu.wpi.first.wpilibj.geometry.Transform2d;
import edu.wpi.first.wpilibj.geometry.Translation2d;

import java.util.ArrayList;
import java.util.List;

public class TransformUtils {
    public static Pose2d translatePose(Pose2d offset, Pose2d pose) {
        return offset.plus(new Transform2d(pose.getTranslation(), pose.getRotation()));
    }

    public static List<Translation2d> translateWaypoints(Pose2d offset, List<Translation2d> waypoints) {
        List<Translation2d> outputWaypoints = new ArrayList<>();
        for (Translation2d waypoint : waypoints) {
            outputWaypoints.add(offset.plus(new Transform2d(waypoint, new Rotation2d())).getTranslation());
        }
        return outputWaypoints;
    }

    public static List<Pose2d> translatePoseList(Pose2d offset, List<Pose2d> waypoints) {
        List<Pose2d> outputWaypoints = new ArrayList<>();
        for (Pose2d waypoint : waypoints) {
            outputWaypoints.add(translatePose(offset, waypoint));
        }
        return outputWaypoints;
    }
}
