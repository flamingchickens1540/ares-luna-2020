package org.team1540.robot2020;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.BaseMotorController;
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
import edu.wpi.first.wpilibj2.command.*;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import java.util.List;
import org.apache.log4j.Logger;
import org.team1540.robot2020.commands.Autonomous;
import org.team1540.robot2020.commands.TestMotors;
import org.team1540.robot2020.commands.cargomech.CargoMechIntake;
import org.team1540.robot2020.commands.cargomech.CargoMechManualControl;
import org.team1540.robot2020.commands.drivetrain.PointDrive;
import org.team1540.robot2020.commands.drivetrain.TankDrive;
import org.team1540.robot2020.shouldbeinrooster.InstCommand;
import org.team1540.robot2020.subsystems.CargoMech;
import org.team1540.robot2020.subsystems.DriveTrain;
import org.team1540.rooster.triggers.DPadAxis;
import org.team1540.rooster.util.ChickenXboxController;
import org.team1540.rooster.wrappers.NavX;

import static org.team1540.rooster.util.ChickenXboxController.XboxButton.*;

public class RobotContainer {

    private static final Logger logger = Logger.getLogger(RobotContainer.class);

    private ChickenXboxController driver = new ChickenXboxController(0);
    private ChickenXboxController copilot = new ChickenXboxController(1);
    private ChickenXboxController test = new ChickenXboxController(2);

    private NavX navx = new NavX(SPI.Port.kMXP);

    private BaseMotorController[] motors;
    private int testMotorI = 0;

    private DriveTrain driveTrain;
    private CargoMech cargoMech;

    public RobotContainer() {
        logger.info("Creating robot container...");

//        initButtonBindings();
//        initModeTransitionBindings();
//        initDefaultCommands();

//        SmartDashboard.putData("drive/resetEncoders", new ResetEncoders(driveTrain));
    }

    public void init(boolean test) {
        driveTrain = new DriveTrain(copilot);
        cargoMech = new CargoMech();

        motors = new BaseMotorController[]{
                driveTrain.driveMotorRightA,
                driveTrain.driveMotorRightB,
                driveTrain.driveMotorRightC,
                driveTrain.driveMotorLeftA,
                driveTrain.driveMotorLeftB,
                driveTrain.driveMotorLeftC,
                cargoMech.cargoRollerTop
        };

        initModeTransitionBindings();
        if (test) {
            initTest();
        } else {
            initDefaultCommands();
            initButtonBindings();
        }
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

    public void initTest() {
        test.getButton(ChickenXboxController.XboxButton.RB).whenPressed(() -> {
            if (testMotorI == motors.length - 1) {
                testMotorI = 0;
            } else {
                testMotorI++;
            }
            System.out.println(testMotorI);
        });
        test.getButton(ChickenXboxController.XboxButton.LB).whenPressed(() -> {
            if (testMotorI == 0) {
                testMotorI = motors.length - 1;
            } else {
                testMotorI--;
            }
            System.out.println(testMotorI);
        });
        driveTrain.setDefaultCommand(new InstCommand(() -> {}, driveTrain).perpetually());
        cargoMech.setDefaultCommand(new InstCommand(() -> {}, cargoMech).perpetually());
        new TestMotors(motors, () -> testMotorI, test.getAxis(ChickenXboxController.XboxAxis.RIGHT_Y).withDeadzone(0.1)).schedule();
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
        return new Autonomous(driveTrain, cargoMech, driver);
    }

    private void initDefaultCommands() {
        driveTrain.setDefaultCommand(new TankDrive(driveTrain, driver));
//        cargoMech.setDefaultCommand(new CargoMechManualControl(cargoMech, copilot.getAxis(ChickenXboxController.XboxAxis.RIGHT_Y)));
    }
}
