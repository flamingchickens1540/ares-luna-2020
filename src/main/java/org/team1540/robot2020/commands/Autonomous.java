package org.team1540.robot2020.commands;

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
import java.util.List;
import org.team1540.robot2020.subsystems.DriveTrain;

public class Autonomous extends SequentialCommandGroup {

    // Feed forward constants
    public static final double ksVolts = 0.669;
    public static final double kvVoltSecondsPerMeter = 2.76;
    public static final double kaVoltSecondsSquaredPerMeter = 0.662;
    // Ramsete PID controllers
//    public final double kPDriveVel = 19.3;
    public static final double kPDriveVel = 1;
    // Motion control
    public static final double kRamseteB = 2;
    public static final double kRamseteZeta = 0.7;
    public static final double kMaxSpeedMetersPerSecond = 2;
    public static final double kMaxAccelerationMetersPerSecondSquared = 1;
    public static final DifferentialDriveKinematics kDriveKinematics = new DifferentialDriveKinematics(DriveTrain.trackwidthMeters);

    private DriveTrain driveTrain;

    public Autonomous(DriveTrain driveTrain) {
        this.driveTrain = driveTrain;

        // Create a voltage constraint to ensure we don't accelerate too fast
        var autoVoltageConstraint =
            new DifferentialDriveVoltageConstraint(
                new SimpleMotorFeedforward(ksVolts,
                    kvVoltSecondsPerMeter,
                    kaVoltSecondsSquaredPerMeter),
                kDriveKinematics,
                10);

        // Create config for trajectory
        TrajectoryConfig config =
            new TrajectoryConfig(kMaxSpeedMetersPerSecond,
                kMaxAccelerationMetersPerSecondSquared)
                // Add kinematics to ensure max speed is actually obeyed
                .setKinematics(kDriveKinematics)
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
                    new Translation2d(1, 1),
                    new Translation2d(2, -1)
//                        new Translation2d(-1, -1),
//                        new Translation2d(-2, -1)
                ),
                // End 3 meters straight ahead of where we started, facing forward
                new Pose2d(3, 0, new Rotation2d(0)),
                // Pass config
                config
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
            // RamseteCommand passes volts to the callback
            driveTrain::tankDriveVolts,
            driveTrain
        );

        addCommands(
                new InstantCommand(() -> driveTrain.resetOdometry(new Pose2d())),
                ramseteCommand,
                new InstantCommand(() -> driveTrain.tankDriveVelocity(0, 0))
        );
    }
}
