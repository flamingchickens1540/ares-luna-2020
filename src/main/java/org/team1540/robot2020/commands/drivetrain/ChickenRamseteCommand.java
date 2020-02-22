package org.team1540.robot2020.commands.drivetrain;

import edu.wpi.first.wpilibj.geometry.Pose2d;
import edu.wpi.first.wpilibj.geometry.Translation2d;
import edu.wpi.first.wpilibj.trajectory.TrajectoryGenerator;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import org.team1540.robot2020.LocalizationManager;
import org.team1540.robot2020.RamseteConfig;
import org.team1540.robot2020.utils.InstCommand;
import org.team1540.robot2020.utils.RamseteUtils;

import java.util.List;
import java.util.function.Supplier;

public class ChickenRamseteCommand extends SequentialCommandGroup {

    public ChickenRamseteCommand(Supplier<Pose2d> offset, Pose2d start, Pose2d end, boolean reversed, LocalizationManager localizationManager, DriveTrain driveTrain) {
        this(offset, start, List.of(), end, reversed, localizationManager, driveTrain);
    }

    public ChickenRamseteCommand(Supplier<Pose2d> offset, Pose2d start, List<Translation2d> waypoints, Pose2d end, boolean reversed, LocalizationManager localizationManager, DriveTrain driveTrain) {

        addCommands(
                new BaseChickenRamseteCommand(
                        () -> {
                            RamseteConfig.trajectoryConfig.setReversed(reversed);
                            return TrajectoryGenerator.generateTrajectory(
                                    RamseteUtils.translatePose(offset.get(), start),
                                    RamseteUtils.translateWaypoints(offset.get(), waypoints),
                                    RamseteUtils.translatePose(offset.get(), end),
                                    RamseteConfig.trajectoryConfig
                            );
                        },
                        localizationManager,
                        driveTrain
                ),
                new InstCommand(() -> driveTrain.setVoltage(0, 0))
        );
    }
}