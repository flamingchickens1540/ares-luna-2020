package org.team1540.robot2020.commands.drivetrain;

import edu.wpi.first.wpilibj.geometry.Pose2d;
import edu.wpi.first.wpilibj.geometry.Rotation2d;
import edu.wpi.first.wpilibj.geometry.Translation2d;
import edu.wpi.first.wpilibj.trajectory.Trajectory;
import edu.wpi.first.wpilibj.trajectory.TrajectoryGenerator;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import org.team1540.robot2020.LocalizationManager;
import org.team1540.robot2020.RamseteConfig;
import org.team1540.robot2020.utils.InstCommand;

import java.util.List;

public class ChickenRamseteCommand extends SequentialCommandGroup {
    private static Pose2d lastPose = new Pose2d(0, 0, new Rotation2d(0));

    public ChickenRamseteCommand(Pose2d end, LocalizationManager localizationManager, DriveTrain driveTrain) {
        this(end, localizationManager, driveTrain, false);
    }

    public ChickenRamseteCommand(Pose2d end, LocalizationManager localizationManager, DriveTrain driveTrain, boolean reversed) {
        this(end, List.of(), localizationManager, driveTrain, reversed);
    }

    public ChickenRamseteCommand(Pose2d end, List<Translation2d> waypoints, LocalizationManager localizationManager, DriveTrain driveTrain) {
        this(end, waypoints, localizationManager, driveTrain, false);
    }

    public ChickenRamseteCommand(Pose2d end, List<Translation2d> waypoints, LocalizationManager localizationManager, DriveTrain driveTrain, boolean reversed) {
        RamseteConfig.trajectoryConfig.setReversed(reversed);

        Trajectory trajectory = TrajectoryGenerator.generateTrajectory(
                lastPose,
                waypoints,
                end,
                RamseteConfig.trajectoryConfig
        );

        lastPose = end;

        addCommands(
                new BaseChickenRamseteCommand(
                        trajectory,
                        localizationManager,
                        driveTrain
                ),
                new InstCommand(() -> driveTrain.setVoltage(0, 0))
        );
    }
}