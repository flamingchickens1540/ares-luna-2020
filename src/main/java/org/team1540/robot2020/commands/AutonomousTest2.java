package org.team1540.robot2020.commands;

import edu.wpi.first.wpilibj.controller.PIDController;
import edu.wpi.first.wpilibj.controller.RamseteController;
import edu.wpi.first.wpilibj.controller.SimpleMotorFeedforward;
import edu.wpi.first.wpilibj.geometry.Pose2d;
import edu.wpi.first.wpilibj.geometry.Rotation2d;
import edu.wpi.first.wpilibj.geometry.Translation2d;
import edu.wpi.first.wpilibj.trajectory.Trajectory;
import edu.wpi.first.wpilibj.trajectory.TrajectoryConfig;
import edu.wpi.first.wpilibj.trajectory.TrajectoryGenerator;
import edu.wpi.first.wpilibj.trajectory.constraint.DifferentialDriveVoltageConstraint;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.RamseteCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import org.team1540.robot2020.RamseteConfig;
import org.team1540.robot2020.commands.drivetrain.DriveTrain;

import java.util.List;

public class AutonomousTest2 extends SequentialCommandGroup {
    public AutonomousTest2(DriveTrain driveTrain) {
        // Create a voltage constraint to ensure we don't accelerate too fast
        var autoVoltageConstraint =
                new DifferentialDriveVoltageConstraint(
                        new SimpleMotorFeedforward(RamseteConfig.ksVolts,
                                RamseteConfig.kvVoltSecondsPerMeter,
                                RamseteConfig.kaVoltSecondsSquaredPerMeter),
                        RamseteConfig.kDriveKinematics,
                        10);

        // Create config for trajectory
        TrajectoryConfig config =
                new TrajectoryConfig(RamseteConfig.kMaxSpeedMetersPerSecond,
                        RamseteConfig.kMaxAccelerationMetersPerSecondSquared)
                        // Add kinematics to ensure max speed is actually obeyed
                        .setKinematics(RamseteConfig.kDriveKinematics)
                        // Apply the voltage constraint
                        .addConstraint(autoVoltageConstraint);

//        Follow splines backwards
//        config.setReversed(true);

        // An example trajectory to follow.  All units in meters.
        Trajectory exampleTrajectory = TrajectoryGenerator.generateTrajectory(
                // Start at the origin facing the +X direction
                new Pose2d(0, 0, new Rotation2d(0)),
                // Pass through these two interior waypoints, making an 's' curve path
                List.of(
//                        new Translation2d(1, 0.25),
//                        new Translation2d(2, -0.25)
//                        new Translation2d(-1, -1),
//                        new Translation2d(-2, -1)
                ),
                // End 3 meters straight ahead of where we started, facing forward
                new Pose2d(3, 1, new Rotation2d(Math.PI / 4)),
                // Pass config
                config
        );

        RamseteCommand ramseteCommand = new RamseteCommand(
                exampleTrajectory,
                driveTrain::getPose,
                new RamseteController(RamseteConfig.kRamseteB, RamseteConfig.kRamseteZeta),
                new SimpleMotorFeedforward(RamseteConfig.ksVolts,
                        RamseteConfig.kvVoltSecondsPerMeter,
                        RamseteConfig.kaVoltSecondsSquaredPerMeter),
                RamseteConfig.kDriveKinematics,
                driveTrain::getWheelSpeeds,
                new PIDController(RamseteConfig.kPDriveVel, 0, 0),
                new PIDController(RamseteConfig.kPDriveVel, 0, 0),
                // RamseteCommand passes volts to the callback
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
