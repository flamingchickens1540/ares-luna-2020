package org.team1540.robot2020.commands.drivetrain;

import edu.wpi.first.wpilibj.controller.PIDController;
import edu.wpi.first.wpilibj.controller.RamseteController;
import edu.wpi.first.wpilibj.controller.SimpleMotorFeedforward;
import edu.wpi.first.wpilibj.geometry.Pose2d;
import edu.wpi.first.wpilibj.kinematics.DifferentialDriveKinematics;
import edu.wpi.first.wpilibj.trajectory.Trajectory;
import edu.wpi.first.wpilibj.trajectory.TrajectoryConfig;
import edu.wpi.first.wpilibj.trajectory.TrajectoryGenerator;
import edu.wpi.first.wpilibj.trajectory.constraint.DifferentialDriveVoltageConstraint;
import edu.wpi.first.wpilibj2.command.RamseteCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;

import java.util.List;

public class FollowRamsetePath extends SequentialCommandGroup {

    // TODO: need class that does odometry and follows a sequence of paths

    // Feed forward constants (determined using FRC Robot Characterization app)
    private static final double ksVolts = 0.669;
    private static final double kvVoltSecondsPerMeter = 2.76;
    private static final double kaVoltSecondsSquaredPerMeter = 0.662;

    // PID controllers
    // TODO why are you using a default P value of 1
    private static final double kPDriveVel = 1;

    // Ramsete tuning constants
    private static final double kRamseteB = 2;
    private static final double kRamseteZeta = 0.7;

    // Motion Constraints
    private static final double maxVelocityMetersPerSecond = 2;
    private static final double maxAccelerationMetersPerSecondSq = 1;
    private static final double maxVoltage = 10;

    // Path Constraints
    private static final DifferentialDriveKinematics kDriveKinematics = new DifferentialDriveKinematics(DriveTrain.kTrackwidthMeters);
    private static final DifferentialDriveVoltageConstraint voltageConstraint =
            new DifferentialDriveVoltageConstraint(
                    new SimpleMotorFeedforward(ksVolts, kvVoltSecondsPerMeter, kaVoltSecondsSquaredPerMeter),
                    kDriveKinematics,
                    maxVoltage);

    public FollowRamsetePath(DriveTrain driveTrain, List<Pose2d> waypoints, boolean reversed) {

        TrajectoryConfig trajectoryConfig =
                new TrajectoryConfig(maxVelocityMetersPerSecond, maxAccelerationMetersPerSecondSq)
                        .setKinematics(kDriveKinematics)
                        .addConstraint(voltageConstraint);

        trajectoryConfig.setReversed(reversed);

        Trajectory exampleTrajectory = TrajectoryGenerator.generateTrajectory(
                waypoints,
                trajectoryConfig
        );

        RamseteCommand ramseteCommand = new RamseteCommand(
                exampleTrajectory,
                driveTrain::getPose,
                new RamseteController(kRamseteB, kRamseteZeta),
                new SimpleMotorFeedforward(ksVolts,
                        kvVoltSecondsPerMeter,
                        maxVoltage),
                kDriveKinematics,
                driveTrain::getWheelSpeeds,
                new PIDController(kPDriveVel, 0, 0),
                new PIDController(kPDriveVel, 0, 0),
                driveTrain::setVoltage,
                driveTrain
        );

        addCommands(ramseteCommand);
    }
}
