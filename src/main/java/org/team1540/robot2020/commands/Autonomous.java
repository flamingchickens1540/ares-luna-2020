package org.team1540.robot2020.commands;

import edu.wpi.first.wpilibj.controller.SimpleMotorFeedforward;
import edu.wpi.first.wpilibj.geometry.Pose2d;
import edu.wpi.first.wpilibj.geometry.Rotation2d;
import edu.wpi.first.wpilibj.geometry.Translation2d;
import edu.wpi.first.wpilibj.trajectory.TrajectoryConfig;
import edu.wpi.first.wpilibj.trajectory.TrajectoryGenerator;
import edu.wpi.first.wpilibj.trajectory.constraint.DifferentialDriveVoltageConstraint;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import org.team1540.robot2020.commands.cargomech.CargoMechIntake;
import org.team1540.robot2020.shouldbeinrooster.BaseChickenRamseteCommand;
import org.team1540.robot2020.shouldbeinrooster.ChickenRamseteCommand;
import org.team1540.robot2020.shouldbeinrooster.ChickenTrajectoryGenerator;
import org.team1540.robot2020.subsystems.CargoMech;
import org.team1540.robot2020.subsystems.DriveTrain;
import org.team1540.rooster.util.ChickenXboxController;

import java.util.List;

public class Autonomous extends SequentialCommandGroup {
    private DriveTrain driveTrain;
    private CargoMech cargoMech;
    private ChickenXboxController driver;

    public Autonomous(DriveTrain driveTrain, CargoMech cargoMech, ChickenXboxController driver) {
        this.driveTrain = driveTrain;
        this.cargoMech = cargoMech;
        this.driver = driver;

        // Create a voltage constraint to ensure we don't accelerate too fast
        var autoVoltageConstraint =
                new DifferentialDriveVoltageConstraint(
                        new SimpleMotorFeedforward(driveTrain.ksVolts,
                                driveTrain.kvVoltSecondsPerMeter,
                                driveTrain.kaVoltSecondsSquaredPerMeter),
                        driveTrain.kDriveKinematics,
                        10);

        // Create config for trajectory
        TrajectoryConfig config =
                new TrajectoryConfig(driveTrain.kMaxSpeedMetersPerSecond,
                        driveTrain.kMaxAccelerationMetersPerSecondSquared)
                        // Add kinematics to ensure max speed is actually obeyed
                        .setKinematics(driveTrain.kDriveKinematics)
                        // Apply the voltage constraint
                        .addConstraint(autoVoltageConstraint);

//        Follow splines backwards
//        config.setReversed(true);

        // An example trajectory to follow.  All units in meters.
//        Trajectory exampleTrajectory = TrajectoryGenerator.generateTrajectory(
//                // Start at the origin facing the +X direction
//                new Pose2d(0, 0, new Rotation2d(0)),
//                // Pass through these two interior waypoints, making an 's' curve path
//                List.of(
//                    new Translation2d(0.9, 0.5),
//                    new Translation2d(1.9, -0.5)
////                        new Translation2d(-1, -1),
////                        new Translation2d(-2, -1)
//                ),
//                // End 3 meters straight ahead of where we started, facing forward
//                new Pose2d(2.9, 0, new Rotation2d(0)),
//                // Pass config
//                config
//        );

//        RamseteCommand ramseteCommand = new RamseteCommand(
//                exampleTrajectory,
//                driveTrain::getPose,
//                new RamseteController(driveTrain.kRamseteB, driveTrain.kRamseteZeta),
//                new SimpleMotorFeedforward(driveTrain.ksVolts,
//                        driveTrain.kvVoltSecondsPerMeter,
//                        driveTrain.kaVoltSecondsSquaredPerMeter),
//                driveTrain.kDriveKinematics,
//                driveTrain::getWheelSpeeds,
//                new PIDController(driveTrain.kPDriveVel, 0, 0),
//                new PIDController(driveTrain.kPDriveVel, 0, 0),
//                // RamseteCommand passes volts to the callback
//                driveTrain::tankDriveVolts,
//                driveTrain
//        );

        addCommands(
                new InstantCommand(() -> driveTrain.resetOdometry(new Pose2d())),
                new ChickenRamseteCommand(
                        ChickenTrajectoryGenerator.generateTrajectory(
                                List.of(
                                        new Translation2d(0.5, 0.1),
                                        new Translation2d(1, -0.1)
                                ),
                                new Pose2d(1.5, 0, new Rotation2d(0)),
                                config
                        ),
                        driveTrain
                ),
                new CargoMechIntake(cargoMech),
                new ChickenRamseteCommand(
                        ChickenTrajectoryGenerator.generateTrajectory(
                                List.of(
                                    new Translation2d(2, 0.1),
                                    new Translation2d(2.5, 0.2)
                                ),
                                new Pose2d(3, 0, new Rotation2d(0)),
                                config
                        ),
                        driveTrain
                )
        );
    }
}
