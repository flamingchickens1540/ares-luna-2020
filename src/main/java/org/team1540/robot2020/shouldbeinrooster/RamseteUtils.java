package org.team1540.robot2020.shouldbeinrooster;

import edu.wpi.first.wpilibj.geometry.Pose2d;
import edu.wpi.first.wpilibj.geometry.Transform2d;
import edu.wpi.first.wpilibj.geometry.Translation2d;

import java.util.ArrayList;
import java.util.List;

public class RamseteUtils {
    public static Pose2d translatePose(Pose2d current, Pose2d dest) {
        Transform2d transform = dest.minus(current);
        return new Pose2d(
                transform.getTranslation(),
                transform.getRotation()
        );
    }

    public static List<Translation2d> translateWaypoints(Pose2d current, List<Translation2d> waypoints) {
        Translation2d currentTranslation = current.getTranslation();
        List<Translation2d> outputWaypoints = new ArrayList<>();
        for (Translation2d waypoint : waypoints) {
            outputWaypoints.add(waypoint.minus(currentTranslation));
        }
        return outputWaypoints;
    }
}
