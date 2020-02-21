package org.team1540.robot2020;

import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.revrobotics.CANSparkMax;
import edu.wpi.first.wpilibj.RobotState;
import edu.wpi.first.wpilibj.geometry.Pose2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.*;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import org.apache.log4j.Logger;
import org.team1540.robot2020.commands.Autonomous;
import org.team1540.robot2020.commands.climber.Climber;
import org.team1540.robot2020.commands.climber.ClimberNonSensorControl;
import org.team1540.robot2020.commands.climber.ClimberSensorSequence;
import org.team1540.robot2020.commands.climber.ClimberSequence;
import org.team1540.robot2020.commands.drivetrain.DriveTrain;
import org.team1540.robot2020.commands.drivetrain.PointDrive;
import org.team1540.robot2020.commands.drivetrain.PointToTarget;
import org.team1540.robot2020.commands.funnel.Funnel;
import org.team1540.robot2020.commands.funnel.FunnelRun;
import org.team1540.robot2020.commands.hood.Hood;
import org.team1540.robot2020.commands.hood.HoodManualControl;
import org.team1540.robot2020.commands.hood.HoodZeroSequence;
import org.team1540.robot2020.commands.indexer.Indexer;
import org.team1540.robot2020.commands.indexer.IndexerBallQueueSequence;
import org.team1540.robot2020.commands.indexer.IndexerManualControl;
import org.team1540.robot2020.commands.intake.Intake;
import org.team1540.robot2020.commands.intake.IntakeRun;
import org.team1540.robot2020.commands.shooter.Shooter;
import org.team1540.robot2020.commands.shooter.ShooterLineUpSequence;
import org.team1540.robot2020.commands.shooter.ShooterManualSetpoint;
import org.team1540.robot2020.utils.ChickenXboxController;
import org.team1540.robot2020.utils.InstCommand;
import org.team1540.rooster.triggers.DPadAxis;

import java.util.ArrayList;
import java.util.List;

import static edu.wpi.first.wpilibj2.command.CommandGroupBase.parallel;
import static org.team1540.robot2020.utils.ChickenXboxController.XboxButton.*;


public class RobotContainer {

    public static final boolean ENABLE_TESTING_CONTROLLERS = true;

    // TODO: logging debugMode variable to avoid putting things to networktables unnecessarily
    // TODO: don't use SmartDashboard, just use the network tables interface
    private static final Logger logger = Logger.getLogger(RobotContainer.class);

    private ChickenXboxController driverController = new ChickenXboxController(0);
    private ChickenXboxController copilotController = new ChickenXboxController(1);
    private ChickenXboxController distanceOffsetTestingController = new ChickenXboxController(2);

    private DriveTrain driveTrain = new DriveTrain();
    private Intake intake = new Intake();
    private Funnel funnel = new Funnel();
    private Indexer indexer = new Indexer();
    private Shooter shooter = new Shooter();
    private Hood hood = new Hood();
    //    private ControlPanel controlPanel = new ControlPanel();
    private Climber climber = new Climber();

    private LocalizationManager localizationManager = new LocalizationManager(driveTrain);


    RobotContainer() {
        logger.info("Creating robot container...");

        initButtonBindings();
        initDefaultCommands();
        initModeTransitionBindings();
        initDashboard();

        // TODO: Replace with a notifier that runs more often than commands
        localizationManager.schedule();

        new WaitCommand(20).andThen(new ConditionalCommand(new InstCommand(true), new InstCommand(() -> {
            logger.info("Turning off limelight LEDs...");
            localizationManager.setLimelightLeds(false);
        }, true), RobotState::isEnabled)).schedule();
    }

    @SuppressWarnings("DanglingJavadoc")
    private void initButtonBindings() {
        logger.info("Initializing button bindings...");

        // Driver
        driverController.getButton(LEFT_BUMPER).whileHeld(new ShooterLineUpSequence(driveTrain, shooter, hood, driverController, localizationManager));
        driverController.getButton(RIGHT_BUMPER).whileHeld(parallel(indexer.commandPercent(1), new FunnelRun(funnel), new IntakeRun(intake)));

        // Copilot

//        copilotController.getButton(Y).whenPressed(localizationManager.getNavX()::zeroYaw);

        Command ballQueueCommand = new IndexerBallQueueSequence(indexer, funnel);
        Command intakeCommand = new IntakeRun(intake).alongWith(new ScheduleCommand(ballQueueCommand));
        copilotController.getButton(A).whenPressed(intakeCommand);
        copilotController.getButton(B).cancelWhenPressed(intakeCommand);
        copilotController.getButton(X).whenPressed(new InstantCommand(() -> funnel.stop(), funnel));
        copilotController.getButton(DPadAxis.DOWN).whileHeld(intake.commandPercent(-1));
        copilotController.getButton(DPadAxis.UP).whileHeld(
                parallel(indexer.commandPercent(-1), funnel.commandPercent(-1, -1))
        );
        copilotController.getButton(Y).whenPressed(new ClimberSensorSequence(climber, copilotController.getAxis(ChickenXboxController.XboxAxis.RIGHT_X)));

        ClimberSequence climberSequence = new ClimberSequence(climber, copilotController.getAxis(ChickenXboxController.XboxAxis.LEFT_TRIG));
        copilotController.getButton(BACK).and(copilotController.getButton(START)).whenActive(() -> {
            if (climberSequence.isScheduled()) {
                climberSequence.cancel();
            }
            climberSequence.schedule();
        });

        copilotController.getButton(START).and(copilotController.getButton(BACK)).whenActive(new ClimberNonSensorControl(
                climber,
                copilotController.getAxis(ChickenXboxController.XboxAxis.LEFT_X),
                copilotController.getButton(START)));

        if (ENABLE_TESTING_CONTROLLERS) {
            // Testing Controller - Distance offset tuning
            distanceOffsetTestingController.getButton(LEFT_BUMPER).whileHeld(indexer.commandPercent(1));

            distanceOffsetTestingController.getButton(B).whenPressed(new IndexerManualControl(indexer,
                    distanceOffsetTestingController.getAxis(ChickenXboxController.XboxAxis.LEFT_Y).withDeadzone(0.1)));

            distanceOffsetTestingController.getButton(Y).whenPressed(new HoodZeroSequence(hood));

            ShooterManualSetpoint shooterManualSetpoint = new ShooterManualSetpoint(shooter,
                    distanceOffsetTestingController.getAxis(ChickenXboxController.XboxAxis.LEFT_X));
            distanceOffsetTestingController.getButton(X).toggleWhenPressed(shooterManualSetpoint);

            List<Double> distanceList = new ArrayList<>();
            List<Double> hoodList = new ArrayList<>();
            List<Double> flywheelList = new ArrayList<>();
            distanceOffsetTestingController.getButton(BACK).whenPressed(new InstantCommand(() -> {
                distanceList.add(localizationManager.getCorrectedLidarDistance());
                hoodList.add(hood.getPosition());
                flywheelList.add(shooterManualSetpoint.getSetpoint());
                SmartDashboard.putNumberArray("distanceOffsetTesting/DISTANCE", distanceList.toArray(new Double[]{}));
                SmartDashboard.putNumberArray("distanceOffsetTesting/HOOD", hoodList.toArray(new Double[]{}));
                SmartDashboard.putNumberArray("distanceOffsetTesting/FLYWHEEL", flywheelList.toArray(new Double[]{}));
            }));

            distanceOffsetTestingController.getButton(START).toggleWhenPressed(new HoodManualControl(hood,
                    distanceOffsetTestingController.getAxis(ChickenXboxController.XboxAxis.RIGHT_X)));

            distanceOffsetTestingController.getButton(A).toggleWhenPressed(new PointToTarget(driveTrain, localizationManager, driverController, true));
        }
    }

    private void initDefaultCommands() {
        logger.info("Initializing default commands...");

        driveTrain.setDefaultCommand(new PointDrive(driveTrain, localizationManager,
                driverController.getAxis2D(ChickenXboxController.Hand.RIGHT),
                driverController.getAxis(ChickenXboxController.XboxAxis.LEFT_X),
                driverController.getButton(ChickenXboxController.XboxButton.Y)));

        intake.setDefaultCommand(intake.commandStop().perpetually());
        funnel.setDefaultCommand(funnel.commandStop().perpetually());
        indexer.setDefaultCommand(indexer.commandStop().perpetually());
        shooter.setDefaultCommand(shooter.commandStop().perpetually());
        hood.setDefaultCommand(hood.commandStop().perpetually());
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
            localizationManager.setLimelightLeds(true);
            logger.info("Mechanism brakes enabled");
        });

        disabled.whenActive(new WaitCommand(2)
                .alongWith(new InstCommand(() -> {
                    localizationManager.setLimelightLeds(false);
                    logger.debug("Disabling mechanism brakes in 2 seconds");
                }, true))
                .andThen(new ConditionalCommand(new InstCommand(true), new InstCommand(() -> {
                    driveTrain.setBrakes(NeutralMode.Coast);
                    intake.setBrake(CANSparkMax.IdleMode.kCoast);
                    indexer.setBrake(NeutralMode.Coast);
                    climber.setBrake(NeutralMode.Coast);
                    logger.info("Mechanism brakes disabled");
                }, true), RobotState::isEnabled)));
    }

    private void initDashboard() {
        SmartDashboard.putData("localizationManager/ResetOdometry", new InstCommand(() -> {
            driveTrain.resetEncoders();
            localizationManager.resetOdometry(new Pose2d());
        }, true));
    }

    Command getAutoCommand() {
        return new Autonomous(driveTrain, intake, funnel, indexer, shooter, hood, climber, localizationManager);
    }
}
