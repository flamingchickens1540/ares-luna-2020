package org.team1540.robot2020;

import edu.wpi.first.wpilibj.RobotState;
import edu.wpi.first.wpilibj2.command.ConditionalCommand;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import org.apache.log4j.Logger;
import org.team1540.robot2020.subsystems.DriveTrain;
import org.team1540.robot2020.util.InstCommand;
import org.team1540.rooster.util.ChickenXboxController;

public class RobotContainer {

    private static final Logger logger = Logger.getLogger(RobotContainer.class);

    private ChickenXboxController driver = new ChickenXboxController(0);
    private ChickenXboxController copilot = new ChickenXboxController(1);

    private DriveTrain driveTrain = new DriveTrain();

    public RobotContainer() {
        logger.info("Creating robot container...");

        initButtonBindings();
        initModeTransitionBindings();
    }

    private void initButtonBindings() {
        logger.info("Initializing button bindings...");

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
}
