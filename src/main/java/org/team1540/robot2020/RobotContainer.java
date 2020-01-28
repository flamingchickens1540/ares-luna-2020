package org.team1540.robot2020;

import edu.wpi.first.wpilibj.RobotState;
import edu.wpi.first.wpilibj.SPI;
import edu.wpi.first.wpilibj.geometry.Pose2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.ConditionalCommand;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import org.apache.log4j.Logger;
import org.team1540.robot2020.commands.Autonomous;
import org.team1540.robot2020.commands.IntakeIndexSequence;
import org.team1540.robot2020.commands.ShootSequence;
import org.team1540.robot2020.commands.drivetrain.PointDrive;
import org.team1540.robot2020.commands.drivetrain.TankDrive;
import org.team1540.robot2020.commands.indexer.IndexerJoystickControl;
import org.team1540.robot2020.subsystems.DriveTrain;
import org.team1540.robot2020.subsystems.Indexer;
import org.team1540.robot2020.subsystems.Intake;
import org.team1540.robot2020.subsystems.Shooter;
import org.team1540.robot2020.utils.ChickenXboxController;
import org.team1540.robot2020.utils.InstCommand;
import org.team1540.robot2020.utils.NavX;

import static org.team1540.robot2020.utils.ChickenXboxController.XboxButton.*;


public class RobotContainer {

    private static final Logger logger = Logger.getLogger(RobotContainer.class);

    private ChickenXboxController driver = new ChickenXboxController(0);
    private ChickenXboxController copilot = new ChickenXboxController(1);

    private NavX navx = new NavX(SPI.Port.kMXP);

    private DriveTrain driveTrain = new DriveTrain();
    private Intake intake = new Intake();
    private Indexer indexer = new Indexer();
    private Shooter shooter = new Shooter();

    public RobotContainer() {
        logger.info("Creating robot container...");

        initButtonBindings();
        initModeTransitionBindings();
        initDefaultCommands();

//        SmartDashboard.putData("drive/resetEncoders", new ResetEncoders(driveTrain));
    }

    private void initButtonBindings() {
        logger.info("Initializing button bindings...");

//        driver.getButton(A).whenPressed(driveTrain::resetEncoders);
//        driver.getButton(X).whenPressed(() -> driveTrain.resetOdometry(new Pose2d()));
//        driver.getButton(B).toggleWhenPressed(new PointDrive(driveTrain, driver));
//        driver.getButton(Y).whenPressed(driveTrain::zeroNavx);
//        driver.getButton(RIGHT_BUMPER).whileHeld(new ShootSequence(intake, indexer, shooter));
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
        driveTrain.setDefaultCommand(new PointDrive(driveTrain, driver));
//        indexer.setDefaultCommand(new IntakeIndexSequence(intake, indexer));
        indexer.setDefaultCommand(new IndexerJoystickControl(indexer, copilot.getAxis(ChickenXboxController.XboxAxis.LEFT_Y)));
    }
}
