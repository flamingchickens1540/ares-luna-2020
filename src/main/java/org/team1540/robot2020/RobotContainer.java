package org.team1540.robot2020;

import com.ctre.phoenix.motorcontrol.NeutralMode;
import edu.wpi.first.wpilibj.I2C;
import edu.wpi.first.wpilibj.RobotState;
import edu.wpi.first.wpilibj.SPI;
import edu.wpi.first.wpilibj.geometry.Pose2d;
import edu.wpi.first.wpilibj.geometry.Rotation2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.ConditionalCommand;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import org.apache.log4j.Logger;
import org.team1540.robot2020.commands.drivetrain.FollowRamsetePath;
import org.team1540.robot2020.commands.drivetrain.TankDrive;
import org.team1540.robot2020.commands.funnel.FunnelManualControl;
import org.team1540.robot2020.commands.hood.HoodManualControl;
import org.team1540.robot2020.commands.indexer.IndexerManualControl;
import org.team1540.robot2020.commands.shooter.ShooterManualControl;
import org.team1540.robot2020.subsystems.*;
import org.team1540.robot2020.utils.ChickenXboxController;
import org.team1540.robot2020.utils.InstCommand;
import org.team1540.robot2020.utils.LIDARLite;
import org.team1540.robot2020.utils.NavX;

import java.util.List;

public class RobotContainer {

    // TODO: logging debugMode variable to avoid putting things to networktables unnecessarily

    private static final Logger logger = Logger.getLogger(RobotContainer.class);

    private ChickenXboxController driver = new ChickenXboxController(0);
    private ChickenXboxController copilot = new ChickenXboxController(1);

    private NavX navx = new NavX(SPI.Port.kMXP);
    private LIDARLite lidar = new LIDARLite(I2C.Port.kOnboard);

    // TODO split Intake into Intake and Funnel
    private DriveTrain driveTrain = new DriveTrain(navx);
    private Intake intake = new Intake();
    private Funnel funnel = new Funnel();
    private Indexer indexer = new Indexer();
    private Shooter shooter = new Shooter();
    private Hood hood = new Hood();
    private Climber climber = new Climber();

    public RobotContainer() {
        logger.info("Creating robot container...");

        initButtonBindings();
        initModeTransitionBindings();
        initDefaultCommands();
        initDashboard();
    }

    private void initButtonBindings() {
        logger.info("Initializing button bindings...");

//        driver.getButton(A).whenPressed(driveTrain::resetEncoders);
//        driver.getButton(X).whenPressed(() -> driveTrain.resetOdometry(new Pose2d()));
//        driver.getButton(B).toggleWhenPressed(new PointDrive(driveTrain, driver));
//        driver.getButton(Y).whenPressed(driveTrain::zeroNavx);
//        driver.getButton(RIGHT_BUMPER).whileHeld(new ShootSequence(intake, indexer, shooter));
//        copilot.getButton(DPadAxis.UP).whileHeld(() -> intake.setPercent(true));
//        copilot.getButton(DPadAxis.DOWN).whileHeld(() -> intake.setPercent(false));
//        copilot.getButton(ChickenXboxController.XboxButton.Y).whenPressed(new ShooterSpinUp(shooter, 100));
//        copilot.getButton(ChickenXboxController.XboxButton.Y).whenPressed(new ShooterSpinUp(shooter));
//        copilot.getButton(ChickenXboxController.XboxButton.A).whenPressed(shooter::disableMotors);

//        copilot.getButton(ChickenXboxController.XboxButton.B).whenPressed(new ZeroHoodSequence(hood));

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
            driveTrain.setBrakes(NeutralMode.Brake);
            climber.setBrake(NeutralMode.Brake);
            logger.info("Mechanism brakes enabled");
        });

        disabled.whenActive(new WaitCommand(2)
                .alongWith(new InstCommand(() -> logger.debug("Disabling mechanism brakes in 2 seconds"), true))
                .andThen(new ConditionalCommand(new InstCommand(true), new InstCommand(() -> {
                    // disable brakes
                    driveTrain.setBrakes(NeutralMode.Coast);
                    climber.setBrake(NeutralMode.Coast);logger.info("Mechanism brakes disabled");
                }, true), RobotState::isEnabled)));
    }

    public Command getAutoCommand() {
        // TODO logic for selecting an auto command
        return new FollowRamsetePath(driveTrain, List.of(new Pose2d(0, 0, new Rotation2d(0)), new Pose2d(3, 0, new Rotation2d(0))), false);
    }

    private void initDashboard() {
        SmartDashboard.putData(new InstCommand(() -> climber.zero(), true));
    }

    private void initDefaultCommands() {
        driveTrain.setDefaultCommand(new TankDrive(driveTrain, driver));

        indexer.setDefaultCommand(new IndexerManualControl(indexer,
                copilot.getAxis(ChickenXboxController.XboxAxis.LEFT_X).withDeadzone(0.1)));
        funnel.setDefaultCommand(new FunnelManualControl(funnel,
                copilot.getAxis(ChickenXboxController.XboxAxis.LEFT_X).withDeadzone(0.1)));

//        climber.setDefaultCommand(new ClimberManualControl(climber,
//                copilot.getAxis(ChickenXboxController.XboxAxis.RIGHT_X).withDeadzone(0.05),
//                copilot.getButton(ChickenXboxController.XboxButton.X)));
//        driveTrain.setDefaultCommand(new PointDrive(driveTrain, navx,
//                driver.getAxis2D(RIGHT),
//                driver.getAxis(LEFT_X).withDeadzone(.1),
//                driver.getButton(Y)
//        ));
//        indexer.setDefaultCommand(new IndexSequence(indexer));
//        intake.setDefaultCommand(new RunIntake(intake, indexer));

        shooter.setDefaultCommand(new ShooterManualControl(shooter,
                copilot.getAxis(ChickenXboxController.XboxAxis.LEFT_TRIG).withDeadzone(0.15)));
        hood.setDefaultCommand(new HoodManualControl(hood,
                copilot.getAxis(ChickenXboxController.XboxAxis.RIGHT_X).withDeadzone(0.15)));
    }
}
