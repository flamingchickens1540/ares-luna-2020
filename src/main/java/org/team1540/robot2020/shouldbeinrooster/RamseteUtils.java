package org.team1540.robot2020.shouldbeinrooster;

import edu.wpi.first.wpilibj.geometry.Pose2d;
import edu.wpi.first.wpilibj.geometry.Transform2d;

public class RamseteUtils {
    public static Pose2d translatePose(Pose2d current, Pose2d dest) {
        Transform2d transform = dest.minus(current);
        return new Pose2d(
                transform.getTranslation(),
                transform.getRotation()
        );
    }
}
