package org.team1540.robot2020;

import com.ctre.phoenix.motorcontrol.NeutralMode;
import edu.wpi.first.wpilibj.RobotState;
import edu.wpi.first.wpilibj.geometry.Pose2d;
import edu.wpi.first.wpilibj.geometry.Rotation2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.ConditionalCommand;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import org.apache.log4j.Logger;
import org.team1540.robot2020.commands.climber.ClimbSequence;
import org.team1540.robot2020.commands.drivetrain.FollowRamsetePath;
import org.team1540.robot2020.commands.drivetrain.PointDrive;
import org.team1540.robot2020.commands.indexer.BallQueueSequence;
import org.team1540.robot2020.commands.intake.RunIntake;
import org.team1540.robot2020.subsystems.*;
import org.team1540.robot2020.utils.ChickenXboxController;
import org.team1540.robot2020.utils.InstCommand;

import java.util.List;

import static org.team1540.robot2020.utils.ChickenXboxController.XboxButton.BACK;

//import org.team1540.robot2020.commands.panel.ControlPanelServoManualControl;

public class RobotContainer {

    // TODO: logging debugMode variable to avoid putting things to networktables unnecessarily
    // TODO: don't use SmartDashboard, just use the network tables interface

    private static final Logger logger = Logger.getLogger(RobotContainer.class);

    private ChickenXboxController driverController = new ChickenXboxController(0);
    private ChickenXboxController copilotController = new ChickenXboxController(1);
    private ChickenXboxController testClimbController = new ChickenXboxController(2);
    private ChickenXboxController testControlPanelController = new ChickenXboxController(3);

    private LocalizationController localizationController = new LocalizationController();

    private DriveTrain driveTrain = new DriveTrain(localizationController.getNavX());
    private Intake intake = new Intake();
    private Funnel funnel = new Funnel();
    private Indexer indexer = new Indexer();
    private Shooter shooter = new Shooter();
    private Hood hood = new Hood();
    //    private ControlPanel controlPanel = new ControlPanel();
    private Climber climber = new Climber();

    RobotContainer() {
        logger.info("Creating robot container...");

        initButtonBindings();
        initModeTransitionBindings();
        initDefaultCommands();
        initDashboard();
    }

    private void initButtonBindings() {
        logger.info("Initializing button bindings...");

        testClimbController.getButton(BACK).whenPressed(new ClimbSequence(climber,
                testClimbController.getAxis(ChickenXboxController.XboxAxis.LEFT_TRIG)));
    }

    private void initDefaultCommands() {
        logger.info("Initializing default commands...");

        driveTrain.setDefaultCommand(new PointDrive(driveTrain, localizationController.getNavX(),
                driverController.getAxis2D(ChickenXboxController.Hand.RIGHT),
                driverController.getAxis(ChickenXboxController.XboxAxis.LEFT_X),
                driverController.getButton(ChickenXboxController.XboxButton.Y)));

//        driveTrain.setDefaultCommand(new PointToTarget(localizationController.getNavX(), driveTrain, driverController, localizationController.getLimelight(), new PIDConfig(0.4, 0.07, 1.0, 0.0025, 0.2, 0.01)));

        intake.setDefaultCommand(new RunIntake(intake));
        indexer.setDefaultCommand(new BallQueueSequence(indexer, funnel));
        shooter.setDefaultCommand(new InstCommand(() -> shooter.setPercent(0), shooter).perpetually());
        hood.setDefaultCommand(new InstCommand(() -> hood.setPercent(0), hood).perpetually());
//        controlPanel.setDefaultCommand();
    }

    private void initModeTransitionBindings() {
        logger.info("Initializing mode transition bindings...");

        var inTeleop = new Trigger(RobotState::isOperatorControl);
        var inAuto = new Trigger(RobotState::isAutonomous);
        var inTest = new Trigger(RobotState::isTest);
        var enabled = new Trigger(RobotState::isEnabled);
        var disabled = new Trigger(RobotState::isDisabled);

        inAuto.whenActive(() -> {
            climber.zero();
        });

        enabled.whenActive(() -> {
            // enable brakes
            driveTrain.setBrakes(NeutralMode.Brake);
            indexer.setBrake(NeutralMode.Brake);
            climber.setBrake(NeutralMode.Brake);
            logger.info("Mechanism brakes enabled");
        });

        disabled.whenActive(new WaitCommand(2)
                .alongWith(new InstCommand(() -> logger.debug("Disabling mechanism brakes in 2 seconds"), true))
                .andThen(new ConditionalCommand(new InstCommand(true), new InstCommand(() -> {
                    // disable brakes
                    driveTrain.setBrakes(NeutralMode.Coast);
                    indexer.setBrake(NeutralMode.Coast);
                    climber.setBrake(NeutralMode.Coast);
                    logger.info("Mechanism brakes disabled");
                }, true), RobotState::isEnabled)));
    }

    Command getAutoCommand() {
        // TODO logic for selecting an auto command
        return new FollowRamsetePath(driveTrain, List.of(new Pose2d(0, 0, new Rotation2d(0)), new Pose2d(3, 0, new Rotation2d(0))), false);
    }

    private void initDashboard() {
        SmartDashboard.putData(new InstCommand(() -> climber.zero(), true));
    }

    // TODO: Use Test mode
//    private void initManualControlCommands() {
//        driveTrain.setDefaultCommand(new TankDrive(driveTrain, driverController));
//        funnel.setDefaultCommand(new FunnelManualControl(funnel,
//                copilotController.getAxis2D(ChickenXboxController.Hand.LEFT).withDeadzone(0.1)));
//        indexer.setDefaultCommand(new IndexerManualControl(indexer,
//                copilotController.getAxis(ChickenXboxController.XboxAxis.LEFT_X).withDeadzone(0.1)));
//        funnel.setDefaultCommand(new FunnelManualControl(funnel,
//                copilotController.getAxis2D(ChickenXboxController.Hand.LEFT).withDeadzone(0.1)));
//        intake.setDefaultCommand(new IntakeManualControl(intake,
//                copilotController.getAxis(ChickenXboxController.XboxAxis.LEFT_X).withDeadzone(0.1)));
//        climber.setDefaultCommand(new ClimberManualControl(climber,
//                testClimbController.getAxis(ChickenXboxController.XboxAxis.LEFT_X),
//                testClimbController.getButton(org.checkerframework.checker.units.qual.A)));
//    }
}
