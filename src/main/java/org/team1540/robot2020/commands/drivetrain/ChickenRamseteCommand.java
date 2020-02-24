package org.team1540.robot2020.commands.drivetrain;

import edu.wpi.first.wpilibj.geometry.Pose2d;
import edu.wpi.first.wpilibj.geometry.Translation2d;
import edu.wpi.first.wpilibj.trajectory.Trajectory;
import edu.wpi.first.wpilibj.trajectory.TrajectoryGenerator;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import org.team1540.robot2020.LocalizationManager;
import org.team1540.robot2020.RamseteConfig;
import org.team1540.robot2020.utils.InstCommand;
import org.team1540.robot2020.utils.TransformUtils;

import java.util.List;
import java.util.function.Supplier;

public class ChickenRamseteCommand extends SequentialCommandGroup {

    public ChickenRamseteCommand(Supplier<Pose2d> offset, Supplier<Pose2d> start, Supplier<List<Translation2d>> waypoints, Supplier<Pose2d> end, boolean reversed, LocalizationManager localizationManager, DriveTrain driveTrain) {
        this(localizationManager, driveTrain, () -> {
            RamseteConfig.trajectoryConfig.setReversed(reversed);
            return TrajectoryGenerator.generateTrajectory(
                    TransformUtils.translatePose(offset.get(), start.get()),
                    TransformUtils.translateWaypoints(offset.get(), waypoints.get()),
                    TransformUtils.translatePose(offset.get(), end.get()),
                    RamseteConfig.trajectoryConfig
            );
        });
    }

    public ChickenRamseteCommand(Supplier<Pose2d> offset, Supplier<List<Pose2d>> waypoints, boolean reversed, LocalizationManager localizationManager, DriveTrain driveTrain) {
        this(localizationManager, driveTrain, () -> {
            RamseteConfig.trajectoryConfig.setReversed(reversed);
            return TrajectoryGenerator.generateTrajectory(
                    TransformUtils.translatePoseList(offset.get(), waypoints.get()),
                    RamseteConfig.trajectoryConfig
            );
        });
    }

    public ChickenRamseteCommand(LocalizationManager localizationManager, DriveTrain driveTrain, Supplier<Trajectory> trajectorySupplier) {
        addCommands(
                new BaseChickenRamseteCommand(
                        trajectorySupplier,
                        localizationManager,
                        driveTrain
                ),
                new InstCommand(() -> driveTrain.setVoltage(0, 0))
        );
    }
}