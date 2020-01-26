package org.team1540.robot2020.commands.drivetrain;

import edu.wpi.first.wpilibj.GenericHID.Hand;
import edu.wpi.first.wpilibj2.command.CommandBase;
import org.team1540.robot2020.subsystems.DriveTrain;
import org.team1540.robot2020.utils.ChickenXboxController;
import org.team1540.rooster.util.ControlUtils;

public class TankDrive extends CommandBase {
    private DriveTrain driveTrain;
    private ChickenXboxController driver;

    public TankDrive(DriveTrain driveTrain, ChickenXboxController driver) {
        this.driveTrain = driveTrain;
        this.driver = driver;

        addRequirements(driveTrain);
    }

    @Override
    public void execute() {
        double triggerThrottle = ControlUtils.deadzone(driver.getAxis(ChickenXboxController.XboxAxis.RIGHT_TRIG).value() - driver.getAxis(ChickenXboxController.XboxAxis.LEFT_TRIG).value(), 0.1);
        double leftSpeed = ControlUtils.deadzone(driver.getRectifiedX(Hand.kLeft), 0.1) + triggerThrottle;
        double rightSpeed = ControlUtils.deadzone(driver.getRectifiedX(Hand.kRight), 0.1) + triggerThrottle;
        driveTrain.tankDrivePercent(leftSpeed, rightSpeed);
    }
}
