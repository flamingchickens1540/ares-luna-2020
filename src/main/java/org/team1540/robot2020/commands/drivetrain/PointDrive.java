package org.team1540.robot2020.commands.drivetrain;

import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.controller.PIDController;
import edu.wpi.first.wpilibj2.command.PIDCommand;
import org.team1540.robot2020.subsystems.DriveTrain;
import org.team1540.rooster.util.ChickenXboxController;
import org.team1540.rooster.wrappers.NavX;

public class PointDrive extends PIDCommand {
    private DriveTrain driveTrain;
    private ChickenXboxController driver;
    private NavX navx;

    public PointDrive(DriveTrain driveTrain, ChickenXboxController driver, NavX navx) {
        super(
                new PIDController(0.4, 0, 0),
                navx::getYawRadians,
                () -> Math.atan2(
                        driver.getRectifiedX(GenericHID.Hand.kRight),
                        driver.getRectifiedY(GenericHID.Hand.kRight)
                ),
                (double output) -> {
                    double leftY = driver.getRectifiedX(GenericHID.Hand.kLeft);
                    driveTrain.tankDrive(-output + leftY, output + leftY);
                },
                driveTrain
        );
        this.driveTrain = driveTrain;
        this.driver = driver;
        this.navx = navx;
    }

    @Override
    public boolean isFinished() {
        return getController().atSetpoint();
    }
}
