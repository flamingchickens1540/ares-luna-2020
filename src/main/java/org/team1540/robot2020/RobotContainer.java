package org.team1540.robot2020;

import static org.team1540.rooster.util.ChickenXboxController.XboxButton.X;
import static org.team1540.rooster.util.ChickenXboxController.XboxButton.Y;

import edu.wpi.first.wpilibj.RobotState;
import edu.wpi.first.wpilibj.SPI;
import edu.wpi.first.wpilibj.controller.PIDController;
import edu.wpi.first.wpilibj.controller.RamseteController;
import edu.wpi.first.wpilibj.controller.SimpleMotorFeedforward;
import edu.wpi.first.wpilibj.geometry.Pose2d;
import edu.wpi.first.wpilibj.geometry.Rotation2d;
import edu.wpi.first.wpilibj.trajectory.Trajectory;
import edu.wpi.first.wpilibj.trajectory.TrajectoryConfig;
import edu.wpi.first.wpilibj.trajectory.TrajectoryGenerator;
import edu.wpi.first.wpilibj.trajectory.constraint.DifferentialDriveVoltageConstraint;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandGroupBase;
import edu.wpi.first.wpilibj2.command.ConditionalCommand;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.RamseteCommand;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import java.util.List;
import org.apache.log4j.Logger;
import org.team1540.robot2020.commands.drivetrain.PIDConfig;
import org.team1540.robot2020.commands.drivetrain.PointToTarget;
import org.team1540.robot2020.shouldbeinrooster.InstCommand;
import org.team1540.robot2020.shouldbeinrooster.NavX;
import org.team1540.robot2020.subsystems.DriveTrain;
import org.team1540.rooster.util.ChickenXboxController;
import org.team1540.rooster.wrappers.Limelight;

public class RobotContainer {

    private static final Logger logger = Logger.getLogger(RobotContainer.class);

    private ChickenXboxController driver = new ChickenXboxController(0);
    private ChickenXboxController copilot = new ChickenXboxController(1);

    private Limelight limelight = new Limelight("limelight-a");

    private NavX navx = new NavX(SPI.Port.kMXP);

    private DriveTrain driveTrain = new DriveTrain();

    public RobotContainer() {
        logger.info("Creating robot container...");

        initButtonBindings();
        initModeTransitionBindings();
        initDefaultCommands();

//        SmartDashboard.putData("drive/resetEncoders", new ResetEncoders(driveTrain));
    }

    private void initButtonBindings() {
        logger.info("Initializing button bindings...");

        driver.getButton(Y).whenPressed(driveTrain::resetEncoders);
        driver.getButton(X).whenPressed(() -> driveTrain.resetOdometry(new Pose2d()));
    }

    private void initModeTransitionBindings() {
        logger.info("Initializing mode transition bindings...");

        var inTeleop = new Trigger(RobotState::isOperatorControl);
        var inAuto = new Trigger(RobotState::isAutonomous);
        var inTest = new Trigger(RobotState::isTest);
        var enabled = new Trigger(RobotState::isEnabled);
        var disabled = new Trigger(RobotState::isDisabled);

        enabled.whenActive(() -> {
            // enable brakes
            logger.info("Mechanism brakes enabled");
        });

        disabled.whenActive(new WaitCommand(2)
            .alongWith(new InstCommand(() -> logger.debug("Disabling mechanism brakes in 2 seconds"), true))
            .andThen(new ConditionalCommand(new InstCommand(true), new InstCommand(() -> {
                // disable brakes
                logger.info("Mechanism brakes disabled");
            }, true), RobotState::isEnabled)));

    }

    public Command getAutoCommand() {

        // Create a voltage constraint to ensure we don't accelerate too fast
        DifferentialDriveVoltageConstraint autoVoltageConstraint =
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

        // An example trajectory to follow.  All units in meters.
        Trajectory exampleTrajectory = TrajectoryGenerator.generateTrajectory(
            // Start at the origin facing the +X direction
            new Pose2d(0, 0, new Rotation2d(0)),
            // Pass through these two interior waypoints, making an 's' curve path
            List.of(
//                new Translation2d(1, 1),
//                new Translation2d(2, -1)
            ),
            // End 3 meters straight ahead of where we started, facing forward
            new Pose2d(3, 0, new Rotation2d(0)),
            // Pass config
            config
        );

        RamseteCommand ramseteCommand = new RamseteCommand(
            exampleTrajectory,
            driveTrain::getPose,
            new RamseteController(driveTrain.kRamseteB, driveTrain.kRamseteZeta),
            new SimpleMotorFeedforward(driveTrain.ksVolts,
                driveTrain.kvVoltSecondsPerMeter,
                driveTrain.kaVoltSecondsSquaredPerMeter),
            driveTrain.kDriveKinematics,
            driveTrain::getWheelSpeeds,
            new PIDController(driveTrain.kPDriveVel, 0, 0),
            new PIDController(driveTrain.kPDriveVel, 0, 0),
            // RamseteCommand passes volts to the callback
            driveTrain::tankDriveVolts,
            driveTrain
        );

        // Run path following command, then stop at the end.
        return CommandGroupBase.sequence(
                new InstantCommand(() -> driveTrain.resetOdometry(new Pose2d())),
                ramseteCommand,
            new InstantCommand(() -> driveTrain.tankDriveVelocity(0, 0))
        );
    }

    private void initDefaultCommands() {
        driveTrain.setDefaultCommand(new PointToTarget(navx, driveTrain, driver, limelight, new PIDConfig(1.0, 0.1, 6.0, 0.06, 0.4, 0.03)));
//        driveTrain.setDefaultCommand(new TankDrive(driveTrain, driver, limelight));
    }
}
