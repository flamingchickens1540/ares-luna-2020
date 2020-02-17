package org.team1540.robot2020.shouldbeinrooster;

import edu.wpi.first.wpilibj.controller.SimpleMotorFeedforward;
import edu.wpi.first.wpilibj.geometry.Pose2d;
import edu.wpi.first.wpilibj.geometry.Rotation2d;
import edu.wpi.first.wpilibj.geometry.Translation2d;
import edu.wpi.first.wpilibj.trajectory.Trajectory;
import edu.wpi.first.wpilibj.trajectory.TrajectoryConfig;
import edu.wpi.first.wpilibj.trajectory.TrajectoryGenerator;
import edu.wpi.first.wpilibj.trajectory.constraint.DifferentialDriveVoltageConstraint;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import org.team1540.robot2020.subsystems.DriveTrain;

import java.util.List;

public class ChickenRamseteCommand extends SequentialCommandGroup {
    private static Pose2d lastPose = new Pose2d(0, 0, new Rotation2d(0));

    private static final DifferentialDriveVoltageConstraint autoVoltageConstraint =
            new DifferentialDriveVoltageConstraint(
                    new SimpleMotorFeedforward(DriveTrain.ksVolts,
                            DriveTrain.kvVoltSecondsPerMeter,
                            DriveTrain.kaVoltSecondsSquaredPerMeter),
                    DriveTrain.kDriveKinematics,
                    10);

    private static TrajectoryConfig config = new TrajectoryConfig(
            DriveTrain.kMaxSpeedMetersPerSecond,
            DriveTrain.kMaxAccelerationMetersPerSecondSquared)
            .setKinematics(DriveTrain.kDriveKinematics)
            .addConstraint(autoVoltageConstraint);

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
        config.setReversed(reversed);

        Trajectory trajectory = TrajectoryGenerator.generateTrajectory(
                lastPose,
                waypoints,
                end,
                config
        );

        lastPose = end;

        addCommands(
                new BaseChickenRamseteCommand(
                        trajectory,
                        driveTrain
                ),
                new InstCommand(driveTrain::stop)
        );
    }
}
