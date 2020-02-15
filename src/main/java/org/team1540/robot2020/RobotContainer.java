package org.team1540.robot2020;

import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.revrobotics.CANSparkMax;
import edu.wpi.first.wpilibj.RobotState;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.*;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import org.apache.log4j.Logger;
import org.team1540.robot2020.commands.climber.Climber;
import org.team1540.robot2020.commands.climber.ClimberSequence;
import org.team1540.robot2020.commands.drivetrain.DriveTrain;
import org.team1540.robot2020.commands.drivetrain.PointDrive;
import org.team1540.robot2020.commands.drivetrain.PointToTarget;
import org.team1540.robot2020.commands.funnel.Funnel;
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

import java.util.ArrayList;
import java.util.List;

import static org.team1540.robot2020.utils.ChickenXboxController.XboxButton.*;


public class RobotContainer {

    // TODO: logging debugMode variable to avoid putting things to networktables unnecessarily
    // TODO: don't use SmartDashboard, just use the network tables interface
    private static final Logger logger = Logger.getLogger(RobotContainer.class);

    private ChickenXboxController driverController = new ChickenXboxController(0);
    private ChickenXboxController copilotController = new ChickenXboxController(1);
    private ChickenXboxController distanceOffsetTestingController = new ChickenXboxController(2);

    private LocalizationManager localizationManager = new LocalizationManager();

    private DriveTrain driveTrain = new DriveTrain(localizationManager.getNavX());
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

        // TODO: Replace with a notifier that runs more often than commands
        new LocalizationManager().schedule();
    }

    @SuppressWarnings("DanglingJavadoc")
    private void initButtonBindings() {
        logger.info("Initializing button bindings...");

        /**************************************************************************************************
         *                                                                                                *
         *               -= Driver =-                                        -= COPILOT =-                *
         *                                                                                                *
         *            ↓------------------------- Shooter Lineup            ↓-------------------------- Climber
         *         BUMPER            BUMPER <--- Run Indexer            BUMPER            BUMPER          *
         *       /--------\       /---------\                         /--------\       /---------\        *
         *     /    (+)   '-----'      Y     \                      /    (+)   '-----'    Y       \       *
         *    /            o O o    X     B   \                   /             o O o   X    B <--\--- Stop intake
         *   /      ↑                  A      |                  /      ↑       ^   ^     A <-----|--- Start intake
         *  |     ← + →         (+)           |                 |    ← + →      '---'--(+)--------|--- Climber
         *  |       ↓    _________            |                 |      ↓     _________            |       *
         *  |          /          \           |                 |          /          \           |       *
         *  |         /            \          |                 |         /            \          |       *
         *  \________/              \________/                  \________/              \________/        *
         *                                                                                                *
         **************************************************************************************************/

        // Driver
        driverController.getButton(LEFT_BUMPER).whileHeld(new ShooterLineUpSequence(driveTrain, shooter, hood, driverController, localizationManager));
        driverController.getButton(RIGHT_BUMPER).whileHeld(indexer.commandPercent(1));

        // Copilot
        Command ballQueueCommand = new IndexerBallQueueSequence(indexer, funnel);
        Command intakeCommand = new IntakeRun(intake).alongWith(new ScheduleCommand(ballQueueCommand));
        copilotController.getButton(A).whenPressed(intakeCommand);
        copilotController.getButton(B).cancelWhenPressed(intakeCommand);
        copilotController.getButton(X).whenPressed(new InstantCommand(() -> funnel.stop(), funnel));

        ClimberSequence climberSequence = new ClimberSequence(climber, copilotController.getAxis(ChickenXboxController.XboxAxis.LEFT_TRIG));
        copilotController.getButton(BACK).and(copilotController.getButton(START)).whenActive(() -> {
            if (climberSequence.isScheduled()) {
                climberSequence.cancel();
            }
            climberSequence.schedule();
        });

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
            distanceList.add(localizationManager.getLidar().getDistance());
            hoodList.add(hood.getPosition());
            flywheelList.add(shooterManualSetpoint.getSetpoint());
            SmartDashboard.putNumberArray("distanceOffsetTesting/DISTANCE", distanceList.toArray(new Double[]{}));
            SmartDashboard.putNumberArray("distanceOffsetTesting/HOOD", hoodList.toArray(new Double[]{}));
            SmartDashboard.putNumberArray("distanceOffsetTesting/FLYWHEEL", flywheelList.toArray(new Double[]{}));
        }));

        distanceOffsetTestingController.getButton(A).toggleWhenPressed(new HoodManualControl(hood,
                distanceOffsetTestingController.getAxis(ChickenXboxController.XboxAxis.RIGHT_X)));

        distanceOffsetTestingController.getButton(A).toggleWhenPressed(new PointToTarget(driveTrain, localizationManager.getNavX(), localizationManager.getLimelight(), driverController, true));
    }

    private void initDefaultCommands() {
        logger.info("Initializing default commands...");

        driveTrain.setDefaultCommand(new PointDrive(driveTrain, localizationManager.getNavX(),
                driverController.getAxis2D(ChickenXboxController.Hand.RIGHT),
                driverController.getAxis(ChickenXboxController.XboxAxis.LEFT_X),
                driverController.getButton(ChickenXboxController.XboxButton.Y)));

        intake.setDefaultCommand(new InstCommand(() -> intake.setPercent(0), intake).perpetually());
        funnel.setDefaultCommand(new InstCommand(() -> intake.setPercent(0), intake).perpetually());
        indexer.setDefaultCommand(new InstCommand(() -> intake.setPercent(0), intake).perpetually());
        shooter.setDefaultCommand(new InstCommand(() -> shooter.setPercent(0), shooter).perpetually());
        hood.setDefaultCommand(new InstCommand(() -> hood.setPercent(0), hood).perpetually());
        climber.setDefaultCommand(new InstCommand(() -> climber.setPercent(0), climber).perpetually());
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
            logger.info("Mechanism brakes enabled");
        });

        disabled.whenActive(new WaitCommand(2)
                .alongWith(new InstCommand(() -> logger.debug("Disabling mechanism brakes in 2 seconds"), true))
                .andThen(new ConditionalCommand(new InstCommand(true), new InstCommand(() -> {
                    driveTrain.setBrakes(NeutralMode.Coast);
                    intake.setBrake(CANSparkMax.IdleMode.kCoast);
                    indexer.setBrake(NeutralMode.Coast);
                    climber.setBrake(NeutralMode.Coast);
                    logger.info("Mechanism brakes disabled");
                }, true), RobotState::isEnabled)));
    }

    Command getAutoCommand() {
        return new InstantCommand(() -> {
            climber.zero();
            climber.setRatchet(Climber.RatchetState.DISENGAGED);
        }).andThen(new HoodZeroSequence(hood));
    }
}
