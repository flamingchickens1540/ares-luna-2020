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
import org.team1540.robot2020.utils.RamseteUtils;

import java.util.List;

public class ChickenRamseteCommand extends SequentialCommandGroup {

    public ChickenRamseteCommand(Pose2d start, Pose2d offset, Pose2d end, boolean reversed, LocalizationManager localizationManager, DriveTrain driveTrain) {
        this(start, offset, end, List.of(), reversed, localizationManager, driveTrain);
    }

    public ChickenRamseteCommand(Pose2d start, Pose2d offset, Pose2d end, List<Translation2d> waypoints, boolean reversed, LocalizationManager localizationManager, DriveTrain driveTrain) {
        RamseteConfig.trajectoryConfig.setReversed(reversed);

        addCommands(
                new BaseChickenRamseteCommand(
                        TrajectoryGenerator.generateTrajectory(
                                start,
                                RamseteUtils.translateWaypoints(start, RamseteUtils.translateWaypoints(offset, waypoints)),
                                RamseteUtils.translatePose(start, RamseteUtils.translatePose(offset, end)),
                                RamseteConfig.trajectoryConfig
                        ),
                        localizationManager,
                        driveTrain
                ),
                new InstCommand(() -> driveTrain.setVoltage(0, 0))
        );
    }
}