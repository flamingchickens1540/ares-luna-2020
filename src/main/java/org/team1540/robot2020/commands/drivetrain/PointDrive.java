package org.team1540.robot2020.commands.drivetrain;

import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.controller.PIDController;
import edu.wpi.first.wpilibj2.command.CommandBase;
import org.team1540.robot2020.subsystems.DriveTrain;
import org.team1540.robot2020.utils.ChickenXboxController;
import org.team1540.robot2020.utils.NavX;
import org.team1540.rooster.util.TrigUtils;
import org.team1540.rooster.wrappers.NavX;

import static org.team1540.robot2020.utils.ChickenXboxController.XboxAxis.LEFT_X;

// TODO: Rewrite with features from 2019
public class PointDrive extends CommandBase {
    private DriveTrain driveTrain;
    private ChickenXboxController driver;
    private PIDController controller;

    public PointDrive(DriveTrain driveTrain, ChickenXboxController driver) {
        this.driveTrain = driveTrain;
        this.driver = driver;
        pidController = new PIDController(0.4,0, 0);
        pidController.setSetpoint(0);
        addRequirements(driveTrain);
    }

    @Override
    public void execute() {
        double currentAngle = driveTrain.getHeading();
        double destAngle = driver.getAxis2D(ChickenXboxController.Hand.RIGHT).angle().value();
        double error = TrigUtils.signedAngleError(destAngle, currentAngle);
        double pidValue = pidController.calculate(error);
        double forward = driver.getAxis(LEFT_X).withDeadzone(0.1).value();
        driveTrain.tankDrivePercent(-pidValue + forward, pidValue + forward);
    }
}