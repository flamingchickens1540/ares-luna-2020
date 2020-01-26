package org.team1540.robot2020.commands.drivetrain;

import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.controller.PIDController;
import edu.wpi.first.wpilibj2.command.CommandBase;
import org.team1540.robot2020.subsystems.DriveTrain;
import org.team1540.robot2020.utils.ChickenXboxController;
import org.team1540.robot2020.utils.NavX;
import org.team1540.rooster.util.ControlUtils;
import org.team1540.rooster.util.TrigUtils;

public class PointDrive extends CommandBase {
    private DriveTrain driveTrain;
    private ChickenXboxController driver;
    private NavX navx;
    private PIDController controller;

    public PointDrive(DriveTrain driveTrain, ChickenXboxController driver, NavX navx) {
        this.driveTrain = driveTrain;
        this.driver = driver;
        this.navx = navx;
        controller = new PIDController(0.4,0, 0);
        controller.setSetpoint(0);
        addRequirements(driveTrain);
    }

    @Override
    public void execute() {
        double currentAngle = -navx.getYawRadians() + driveTrain.getNavxOffset();
        double destAngle = Math.atan2(
                ControlUtils.deadzone(driver.getRectifiedX(GenericHID.Hand.kRight), 0.5),
                ControlUtils.deadzone(driver.getRectifiedY(GenericHID.Hand.kRight), 0.5)
        );
        double error = TrigUtils.signedAngleError(destAngle, currentAngle);
        double pidValue = controller.calculate(error);
        double forward = ControlUtils.deadzone(driver.getRectifiedX(GenericHID.Hand.kLeft), 0.1);
        driveTrain.tankDrivePercent(-pidValue + forward, pidValue + forward);
    }

    @Override
    public boolean isFinished() {
        return false;
    }
}