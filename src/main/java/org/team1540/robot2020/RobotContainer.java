package org.team1540.robot2020;

import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.revrobotics.CANSparkMax;
import edu.wpi.first.wpilibj.RobotState;
import edu.wpi.first.wpilibj.geometry.Pose2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.*;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import org.apache.log4j.Logger;
import org.team1540.robot2020.commands.climber.Climber;
import org.team1540.robot2020.commands.climber.ClimberSequenceNoSensor;
import org.team1540.robot2020.commands.drivetrain.*;
import org.team1540.robot2020.commands.funnel.Funnel;
import org.team1540.robot2020.commands.hood.Hood;
import org.team1540.robot2020.commands.hood.HoodSetPositionContinuous;
import org.team1540.robot2020.commands.hood.HoodZeroSequence;
import org.team1540.robot2020.commands.indexer.Indexer;
import org.team1540.robot2020.commands.indexer.IndexerBallQueueSequence;
import org.team1540.robot2020.commands.intake.Intake;
import org.team1540.robot2020.commands.shooter.ShootRapid;
import org.team1540.robot2020.commands.shooter.Shooter;
import org.team1540.robot2020.commands.shooter.ShooterSetVelocityContinuous;
import org.team1540.robot2020.utils.ChickenXboxController;
import org.team1540.robot2020.utils.InstCommand;
import org.team1540.rooster.wrappers.RevBlinken;

import static edu.wpi.first.wpilibj2.command.CommandGroupBase.parallel;
import static org.team1540.robot2020.utils.ChickenXboxController.XboxButton.*;


public class RobotContainer {
    // TODO: logging debugMode variable to avoid putting things to networktables unnecessarily
    // TODO: don't use SmartDashboard, just use the network tables interface
    private static final Logger logger = Logger.getLogger(RobotContainer.class);

    // Valid values are 3 or 6
    private final int auto = 6;
    // Controllers
    private final ChickenXboxController driverController = new ChickenXboxController(0);
    private final ChickenXboxController copilotController = new ChickenXboxController(1);
    // Subsystems
    private final DriveTrain driveTrain = new DriveTrain();
    private final Intake intake = new Intake();
    private final Funnel funnel = new Funnel();
    private final Indexer indexer = new Indexer();
    private final Shooter shooter = new Shooter();
    private final Hood hood = new Hood();
    private final Climber climber = new Climber();
    private final RevBlinken leds = new RevBlinken(0);
    private final LocalizationManager localizationManager = new LocalizationManager(driveTrain, shooter, hood, this::zeroHoodIfFlag);
    private final PointDrive pointDrive = new PointDrive(driveTrain, localizationManager,
            driverController.getAxis2D(ChickenXboxController.Hand.RIGHT),
            driverController.getAxis(ChickenXboxController.XboxAxis.LEFT_X),
            driverController.getButton(ChickenXboxController.XboxButton.Y));

    // Autos
    private final Command autoSixBall = new AutoSixBall(driveTrain, intake, funnel, indexer, shooter, hood, climber, localizationManager, driverController);
    private final Command autoThreeBall = new AutoThreeBall(driveTrain, intake, funnel, indexer, shooter, hood, climber, localizationManager, driverController);

    RobotContainer() {
        logger.info("Creating robot container...");

        initButtonBindings();
        initDefaultCommands();
        initModeTransitionBindings();
        initDashboard();

        // TODO: Replace with a notifier that runs more often than commands
        localizationManager.schedule();
        localizationManager.setOnNavxZeroCallback(pointDrive::zeroAngle);

        SmartDashboard.putNumber("AutoSelector/SelectedBallNumber", auto);
    }

    private void initButtonBindings() {
        logger.info("Initializing button bindings...");

        SmartDashboard.putNumber("robotContainer/shootIndexDistance", 0.11);

        // Commands
        Command ballQueueCommand = new IndexerBallQueueSequence(indexer, funnel, true);
        Command intakeCommand = intake.commandPercent(1).alongWith(new ScheduleCommand(ballQueueCommand));

        // Driver
        driverController.getAxis2D(ChickenXboxController.Hand.RIGHT).magnitude().button(0.7).whileHeld(pointDrive);

        driverController.getButton(START).whileHeld(driveTrain.commandStop().alongWith(hood.commandStop()));
        Command flywheelSpinUp = new ShooterSetVelocityContinuous(shooter, localizationManager::getShooterRPMForSelectedGoal);
        Command shootSequence = new ShootRapid(intake, funnel, indexer, localizationManager, localizationManager::isLinedUp);
        driverController.getButton(LEFT_BUMPER).whileHeld(shootSequence.alongWith(flywheelSpinUp));

        // TODO: Delete this
        driverController.getButton(RIGHT_BUMPER).whenPressed(intakeCommand);
//        driverController.getButton(B).cancelWhenPressed(intakeCommand);
//        driverController.getButton(X).cancelWhenPressed(ballQueueCommand);

        Trigger emergencyDisable = driverController.getButton(LEFT_BUMPER)
                .and(driverController.getButton(RIGHT_BUMPER))
                .and(driverController.getButton(RIGHT_PRESS))
                .and(driverController.getButton(BACK))
                .and(driverController.getButton(START));
        emergencyDisable.whenActive(new InstCommand(() -> {
            throw new NullPointerException("=+=+=+=+= Emergency Disable Triggered! =+=+=+=+=");
        }));

        // Copilot
        copilotController.getButton(A).whenPressed(intakeCommand);
        copilotController.getButton(B).cancelWhenPressed(intakeCommand);
        copilotController.getButton(X).whenPressed(new InstantCommand(funnel::stop, funnel));
        copilotController.getButton(LEFT_BUMPER).whileHeld(intake.commandPercent(-1));
        copilotController.getButton(RIGHT_BUMPER).whileHeld(
                parallel(indexer.commandPercent(-1), funnel.commandPercent(-1, -1), new InstCommand(intakeCommand::cancel))
        );
        copilotController.getButton(BACK).and(copilotController.getButton(START)).whenActive(new ClimberSequenceNoSensor(climber, copilotController.getAxis(ChickenXboxController.XboxAxis.RIGHT_X), copilotController.getButton(BACK)));
        copilotController.getButton(ChickenXboxController.XboxButton.LEFT_PRESS).whileHeld(() -> {
            Hood.offset += copilotController.getAxis(ChickenXboxController.XboxAxis.LEFT_X).value() / 200;
            SmartDashboard.putNumber("hood/offset", Hood.offset);
        });
    }

    private void initDefaultCommands() {
        logger.info("Initializing default commands...");
        driveTrain.setDefaultCommand(new LineUpSequence(driveTrain, indexer, shooter, hood, driverController, localizationManager, true, false).perpetually());
        intake.setDefaultCommand(intake.commandStop().perpetually());
        funnel.setDefaultCommand(funnel.commandStop().perpetually());
        indexer.setDefaultCommand(indexer.commandStop().perpetually());
        shooter.setDefaultCommand(shooter.commandStop().perpetually());
        hood.setDefaultCommand(new HoodSetPositionContinuous(hood, localizationManager::getHoodTicksForSelectedGoal));
        climber.setDefaultCommand(climber.commandStop().perpetually());
    }

    private void initModeTransitionBindings() {
        logger.info("Initializing mode transition bindings...");

        // TODO: Figure out why inTeleop inAuto and inTest are broken
        var enabled = new Trigger(RobotState::isEnabled);
        var disabled = new Trigger(RobotState::isDisabled);

        enabled.whenActive(() -> {
            driveTrain.setBrakes(NeutralMode.Brake);
            intake.setBrake(CANSparkMax.IdleMode.kBrake);
            indexer.setBrake(NeutralMode.Brake);
            climber.setBrake(NeutralMode.Brake);
            hood.setBrake(CANSparkMax.IdleMode.kBrake);
            logger.info("Mechanism brakes enabled");
            leds.set(RevBlinken.ColorPattern.AQUA);
            climber.setRatchet(Climber.RatchetState.DISENGAGED);
        });

        disabled.whenActive(new WaitCommand(2)
                .alongWith(new InstCommand(() -> {
                    leds.set(RevBlinken.ColorPattern.FIRE_LARGE);
                    logger.debug("Disabling mechanism brakes in 2 seconds");
                }, true))
                .andThen(new ConditionalCommand(new InstCommand(true), new InstCommand(() -> {
                    driveTrain.setBrakes(NeutralMode.Coast);
                    intake.setBrake(CANSparkMax.IdleMode.kCoast);
                    indexer.setBrake(NeutralMode.Coast);
                    climber.setBrake(NeutralMode.Coast);
                    hood.setBrake(CANSparkMax.IdleMode.kCoast);
                    logger.info("Mechanism brakes disabled");
                }, true), RobotState::isEnabled)));
    }

    private void initDashboard() {
        SmartDashboard.putData("localizationManager/ResetOdometry", new InstCommand(() -> {
            driveTrain.resetEncoders();
            localizationManager.resetOdometry(new Pose2d());
        }, true));
        SmartDashboard.putNumber("hood/offset", Hood.offset);
    }

    Command getAutoCommand() {
        double selectedAuto = SmartDashboard.getNumber("AutoSelector/SelectedBallNumber", auto);
        double currentYaw = Math.toDegrees(localizationManager.getYawRadians());

        // Use 3 ball auto if the yaw is wrong
        if (currentYaw <= 5) {
            logger.warn("WARNING: Yaw (" + currentYaw + " deg) is wrong, falling back to 3 ball auto");
            return autoThreeBall;
        }

        if (selectedAuto == 6) {
            logger.info("Using 6 ball auto");
            return autoSixBall;
        } else { // If value isn't 6, use 3
            logger.info("Using 3 ball auto");
            if (selectedAuto != 3) {
                logger.warn("WARNING: Unexpected AutoSelector/SelectedBallNumber value (must be 3 or 6), falling back to 3 ball");
            }
            return autoThreeBall;
        }
    }

    public void zeroHoodIfFlag(boolean overrideFlag) {
        if (hood.zeroFlag || overrideFlag) {
            new HoodZeroSequence(hood).schedule();
            hood.zeroFlag = false;
        }
    }
}
