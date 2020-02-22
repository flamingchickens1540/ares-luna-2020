package org.team1540.robot2020.utils;

import edu.wpi.first.wpilibj.geometry.Pose2d;
import edu.wpi.first.wpilibj.geometry.Transform2d;
import edu.wpi.first.wpilibj.geometry.Translation2d;

import java.util.ArrayList;
import java.util.List;

public class RamseteUtils {
    public static Pose2d translatePose(Pose2d start, Pose2d pose) {
        Transform2d transform = pose.minus(start);
        return new Pose2d(
                transform.getTranslation(),
                transform.getRotation()
        );
    }

    public static List<Translation2d> translateWaypoints(Pose2d start, List<Translation2d> waypoints) {
        List<Translation2d> outputWaypoints = new ArrayList<>();
        Translation2d startTranslation = start.getTranslation();
        for (Translation2d waypoint : waypoints) {
            outputWaypoints.add(waypoint.minus(startTranslation));
        }
        return outputWaypoints;
    }
}
