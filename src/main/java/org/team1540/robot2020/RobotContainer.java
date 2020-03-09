package org.team1540.robot2020;

import com.ctre.phoenix.motorcontrol.can.BaseMotorController;
import edu.wpi.first.wpilibj.RobotState;
import edu.wpi.first.wpilibj.SPI;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.ConditionalCommand;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import org.apache.log4j.Logger;
import org.team1540.robot2020.commands.Autonomous;
import org.team1540.robot2020.commands.drivetrain.TankDrive;
import org.team1540.robot2020.shouldbeinrooster.InstCommand;
import org.team1540.robot2020.shouldbeinrooster.MotorTesting;
import org.team1540.robot2020.shouldbeinrooster.SubsystemBase;
import org.team1540.robot2020.subsystems.CargoMech;
import org.team1540.robot2020.subsystems.DriveTrain;
import org.team1540.rooster.util.ChickenXboxController;
import org.team1540.rooster.wrappers.NavX;

public class RobotContainer {

    private static final Logger logger = Logger.getLogger(RobotContainer.class);

    private ChickenXboxController driver = new ChickenXboxController(0);
    private ChickenXboxController copilot = new ChickenXboxController(1);
    private ChickenXboxController test = new ChickenXboxController(2);

    private NavX navx = new NavX(SPI.Port.kMXP);

    private DriveTrain driveTrain = new DriveTrain();
    private CargoMech cargoMech = new CargoMech();

    private SubsystemBase[] subsystems = new SubsystemBase[]{
            driveTrain,
            cargoMech
    };

    Command tankDrive = new TankDrive(driveTrain, driver);

    public RobotContainer() {
        logger.info("Creating robot container...");

        initDefaultCommands();
        initButtonBindings();
        initModeTransitionBindings();

//        SmartDashboard.putData("drive/resetEncoders", new ResetEncoders(driveTrain));
    }

    public void configMotors() {
        System.out.println("Configuring Motors ------------------------------------------");
        for (SubsystemBase subsystem : subsystems) {
            subsystem.configMotors();
        }
    }

    public void testMotors() {
        MotorTesting motorTesting = MotorTesting.getInstance();
        test.getButton(ChickenXboxController.XboxButton.RB).whenPressed(() -> {
            if (motorTesting.hasMotor(motorTesting.testIndex + 1)) {
                motorTesting.testIndex++;
                motorTesting.indexToShuffleboard();
            }
        });
        test.getButton(ChickenXboxController.XboxButton.LB).whenPressed(() -> {
            if (motorTesting.testIndex != 0) {
                motorTesting.testIndex--;
                motorTesting.indexToShuffleboard();
            }
        });
        MotorTesting.getInstance().testMotors(test.getAxis(ChickenXboxController.XboxAxis.RIGHT_Y).withDeadzone(0.1));
    }

    private void initButtonBindings() {
        logger.info("Initializing button bindings...");

//        driver.getButton(A).whenPressed(driveTrain::resetEncoders);
//        driver.getButton(B).whenPressed(() -> driveTrain.resetOdometry(new Pose2d()));
//        driver.getButton(B).toggleWhenPressed(new PointDrive(driveTrain, driver, navx));
//        driver.getButton().whenPressed(driveTrain::zeroNavx);
//        driver.getButton(Y).whenPressed(new CargoMechIntake(cargoMech));
//        copilot.getButton(Y).whileHeld(new StartEndCommand(() -> driveTrain.driveMotorRightA.set(ControlMode.PercentOutput, 0.2), () -> driveTrain.driveMotorRightA.set(ControlMode.PercentOutput, 0), driveTrain));
//        copilot.getButton(A).whileHeld(new StartEndCommand(() -> driveTrain.driveMotorRightB.set(ControlMode.PercentOutput, 0.2), () -> driveTrain.driveMotorRightB.set(ControlMode.PercentOutput, 0), driveTrain));
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

    private void initDefaultCommands() {
        driveTrain.setDefaultCommand(driveTrain.commandStop().perpetually());
        cargoMech.setDefaultCommand(cargoMech.commandStop().perpetually());
//        driveTrain.setDefaultCommand(new TankDrive(driveTrain, driver));
//        cargoMech.setDefaultCommand(new CargoMechManualControl(cargoMech, copilot.getAxis(ChickenXboxController.XboxAxis.RIGHT_Y)));
    }

    public Command getAutoCommand() {
        return new Autonomous(driveTrain, cargoMech, driver);
    }

    public Command getTankDrive() {
        return tankDrive;
    }
}
