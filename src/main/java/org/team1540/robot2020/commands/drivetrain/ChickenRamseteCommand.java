package org.team1540.robot2020.commands.drivetrain;

import edu.wpi.first.wpilibj.geometry.Pose2d;
import edu.wpi.first.wpilibj.geometry.Rotation2d;
import edu.wpi.first.wpilibj.geometry.Translation2d;
import edu.wpi.first.wpilibj.trajectory.Trajectory;
import edu.wpi.first.wpilibj.trajectory.TrajectoryGenerator;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import org.team1540.robot2020.RamseteConfig;
import org.team1540.robot2020.utils.InstCommand;

import java.util.List;

public class ChickenRamseteCommand extends SequentialCommandGroup {
    private static Pose2d lastPose = new Pose2d(0, 0, new Rotation2d(0));

    public ChickenRamseteCommand(Pose2d end, DriveTrain driveTrain) {
        this(end, driveTrain, false);
    }

    public ChickenRamseteCommand(Pose2d end, DriveTrain driveTrain, boolean reversed) {
        this(end, List.of(), driveTrain, reversed);
    }

    public ChickenRamseteCommand(Pose2d end, List<Translation2d> waypoints, DriveTrain driveTrain) {
        this(end, waypoints, driveTrain, false);
    }

    public ChickenRamseteCommand(Pose2d end, List<Translation2d> waypoints, DriveTrain driveTrain, boolean reversed) {
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
                        driveTrain
                ),
                new InstCommand(() -> driveTrain.setVoltage(0, 0))
        );
    }
}