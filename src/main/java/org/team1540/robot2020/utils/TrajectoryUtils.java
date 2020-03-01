package org.team1540.robot2020.utils;

import edu.wpi.first.wpilibj.trajectory.TrajectoryConfig;
import org.team1540.robot2020.RamseteConfig;

public class TrajectoryUtils {
    public static TrajectoryConfig generateTrajectoryConfig(double maxSpeed, double maxAccel, boolean reversed) {
        TrajectoryConfig config = new TrajectoryConfig(
                maxSpeed,
                maxAccel)
                .setKinematics(RamseteConfig.kDriveKinematics)
                .addConstraint(RamseteConfig.autoVoltageConstraint);
        config.setReversed(reversed);
        return config;
    }
}
