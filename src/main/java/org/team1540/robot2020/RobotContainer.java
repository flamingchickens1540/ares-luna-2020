package org.team1540.robot2020;

import edu.wpi.first.wpilibj.RobotState;
import edu.wpi.first.wpilibj.SPI;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.ConditionalCommand;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import org.apache.log4j.Logger;
import org.team1540.robot2020.commands.Autonomous;
import org.team1540.robot2020.commands.climber.ClimberManualControl;
import org.team1540.robot2020.commands.drivetrain.TankDrive;
import org.team1540.robot2020.commands.indexer.IndexerManualControl;
import org.team1540.robot2020.commands.shooter.ShooterSpinUp;
import org.team1540.robot2020.subsystems.*;
import org.team1540.robot2020.utils.ChickenXboxController;
import org.team1540.robot2020.utils.InstCommand;
import org.team1540.robot2020.utils.NavX;
import org.team1540.rooster.triggers.DPadAxis;

public class RobotContainer {

    // TODO add networktables logging for all mechanism state (positions, velocities, current draw)

    private static final Logger logger = Logger.getLogger(RobotContainer.class);

    private ChickenXboxController driver = new ChickenXboxController(0);
    private ChickenXboxController copilot = new ChickenXboxController(1);

    private NavX navx = new NavX(SPI.Port.kMXP);

    private DriveTrain driveTrain = new DriveTrain();
    private Intake intake = new Intake();
    private Indexer indexer = new Indexer();
    private Shooter shooter = new Shooter();
    private Climber climber = new Climber();

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
        copilot.getButton(DPadAxis.UP).whileHeld(() -> intake.setFunnelAndRollerPercent(true));
        copilot.getButton(DPadAxis.DOWN).whileHeld(() -> intake.setFunnelAndRollerPercent(false));
        copilot.getButton(ChickenXboxController.XboxButton.Y).whenPressed(new ShooterSpinUp(shooter, 100));
        copilot.getButton(ChickenXboxController.XboxButton.A).whenPressed(shooter::stop);
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
        indexer.setDefaultCommand(new IndexerManualControl(indexer,
                copilot.getAxis(ChickenXboxController.XboxAxis.LEFT_Y)));
        climber.setDefaultCommand(new ClimberManualControl(climber,
                copilot.getAxis(ChickenXboxController.XboxAxis.RIGHT_Y),
                copilot.getButton(ChickenXboxController.XboxButton.X)));
//        driveTrain.setDefaultCommand(new PointDrive(driveTrain, driver));
//        indexer.setDefaultCommand(new IndexSequence(indexer));
//        intake.setDefaultCommand(new RunIntake(intake, indexer));
    }
}
