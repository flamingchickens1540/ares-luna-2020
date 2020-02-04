package org.team1540.robot2020.commands.drivetrain;

import edu.wpi.first.wpilibj.controller.PIDController;
import edu.wpi.first.wpilibj.controller.RamseteController;
import edu.wpi.first.wpilibj.controller.SimpleMotorFeedforward;
import edu.wpi.first.wpilibj.geometry.Pose2d;
import edu.wpi.first.wpilibj.geometry.Rotation2d;
import edu.wpi.first.wpilibj.geometry.Translation2d;
import edu.wpi.first.wpilibj.kinematics.DifferentialDriveKinematics;
import edu.wpi.first.wpilibj.trajectory.Trajectory;
import edu.wpi.first.wpilibj.trajectory.TrajectoryConfig;
import edu.wpi.first.wpilibj.trajectory.TrajectoryGenerator;
import edu.wpi.first.wpilibj.trajectory.constraint.DifferentialDriveVoltageConstraint;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.RamseteCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import org.team1540.robot2020.subsystems.DriveTrain;

import java.util.List;

public class FollowRamsetePath extends SequentialCommandGroup {

    // TODO: need class that does odometry and follows a sequence of paths

    // Feed forward constants
    private static final double ksVolts = 0.669;
    private static final double kvVoltSecondsPerMeter = 2.76;
    private static final double kaVoltSecondsSquaredPerMeter = 0.662;
    // Ramsete PID controllers
    private static final double kPDriveVel = 1;
    // Motion control
    private static final double kRamseteB = 2;
    private static final double kRamseteZeta = 0.7;
    private static final double kMaxSpeedMetersPerSecond = 2;
    private static final double kMaxAccelerationMetersPerSecondSquared = 1;
    private static final DifferentialDriveKinematics kDriveKinematics = new DifferentialDriveKinematics(DriveTrain.kTrackwidthMeters);

    public FollowRamsetePath(DriveTrain driveTrain) {

        var voltageConstraint =
                new DifferentialDriveVoltageConstraint(
                        new SimpleMotorFeedforward(ksVolts,
                                kvVoltSecondsPerMeter,
                                kaVoltSecondsSquaredPerMeter),
                        kDriveKinematics,
                        10);

        TrajectoryConfig trajectoryConfig =
                new TrajectoryConfig(kMaxSpeedMetersPerSecond,
                        kMaxAccelerationMetersPerSecondSquared)
                        .setKinematics(kDriveKinematics)
                        .addConstraint(voltageConstraint);

//        Follow splines backwards
//        config.setReversed(true);

        // An example trajectory to follow.  All units in meters.
        Trajectory exampleTrajectory = TrajectoryGenerator.generateTrajectory(
                new Pose2d(0, 0, new Rotation2d(0)),
                List.of(
                        new Translation2d(1, 1),
                        new Translation2d(2, -1)
//                        new Translation2d(-1, -1),
//                        new Translation2d(-2, -1)
                ),
                new Pose2d(3, 0, new Rotation2d(0)),
                trajectoryConfig
        );

        RamseteCommand ramseteCommand = new RamseteCommand(
                exampleTrajectory,
                driveTrain::getPose,
                new RamseteController(kRamseteB, kRamseteZeta),
                new SimpleMotorFeedforward(ksVolts,
                        kvVoltSecondsPerMeter,
                        kaVoltSecondsSquaredPerMeter),
                kDriveKinematics,
                driveTrain::getWheelSpeeds,
                new PIDController(kPDriveVel, 0, 0),
                new PIDController(kPDriveVel, 0, 0),
                driveTrain::setVoltage,
                driveTrain
        );

        addCommands(
                new InstantCommand(() -> driveTrain.resetOdometry(new Pose2d())),
                ramseteCommand,
                new InstantCommand(() -> driveTrain.setVelocityMetersPerSecond(0, 0))
        );
    }
}
