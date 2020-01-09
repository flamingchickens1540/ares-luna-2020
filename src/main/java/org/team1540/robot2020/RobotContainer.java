package org.team1540.robot2020;

import edu.wpi.first.wpilibj.RobotState;
import edu.wpi.first.wpilibj.SPI;
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
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandGroupBase;
import edu.wpi.first.wpilibj2.command.ConditionalCommand;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.RamseteCommand;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import java.util.List;
import org.apache.log4j.Logger;
import org.team1540.robot2020.commands.Autonomous;
import org.team1540.robot2020.commands.drivetrain.PointDrive;
import org.team1540.robot2020.commands.drivetrain.TankDrive;
import org.team1540.robot2020.shouldbeinrooster.InstCommand;
import org.team1540.robot2020.subsystems.DriveTrain;
import org.team1540.rooster.util.ChickenXboxController;
import org.team1540.rooster.wrappers.NavX;

import static org.team1540.rooster.util.ChickenXboxController.XboxButton.*;

public class RobotContainer {

    private static final Logger logger = Logger.getLogger(RobotContainer.class);

    private ChickenXboxController driver = new ChickenXboxController(0);
    private ChickenXboxController copilot = new ChickenXboxController(1);


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

        driver.getButton(A).whenPressed(driveTrain::resetEncoders);
        driver.getButton(X).whenPressed(() -> driveTrain.resetOdometry(new Pose2d()));
        driver.getButton(B).toggleWhenPressed(new PointDrive(driveTrain, driver, navx));
        driver.getButton(Y).whenPressed(driveTrain::zeroNavx);
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
        return new Autonomous(driveTrain);
    }

    private void initDefaultCommands() {
        driveTrain.setDefaultCommand(new TankDrive(driveTrain, driver));
    }
}
