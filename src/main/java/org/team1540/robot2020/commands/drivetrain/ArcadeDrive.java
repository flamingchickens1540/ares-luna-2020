package org.team1540.robot2020.commands.drivetrain;

import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj2.command.CommandBase;
import org.team1540.robot2020.subsystems.DriveTrain;
import org.team1540.rooster.util.ChickenXboxController;
import org.team1540.rooster.util.ControlUtils;

public class ArcadeDrive extends CommandBase {
    private DriveTrain driveTrain;
    private ChickenXboxController driver;

    public ArcadeDrive(DriveTrain driveTrain, ChickenXboxController driver) {
        this.driveTrain = driveTrain;
        this.driver = driver;

        addRequirements(driveTrain);
    }

    private double positivePart(double input) {
        if (input <= 0) {
            return 0;
        } else {
            return input;
        }
    }
    private double negativePart(double input) {
        if (input >= 0) {
            return 0;
        } else {
            return input;
        }
    }

    @Override
    public void execute() {
        double triggerThrottle = ControlUtils.deadzone(driver.getTriggerAxis(GenericHID.Hand.kRight) - driver.getTriggerAxis(GenericHID.Hand.kLeft), 0.1);
        double leftY = ControlUtils.deadzone(driver.getRectifiedX(GenericHID.Hand.kLeft), 0.1);
        double rightX = ControlUtils.deadzone(driver.getRectifiedY(GenericHID.Hand.kRight), 0.1);
        double throttleLeft = leftY - (leftY * negativePart(rightX)) + triggerThrottle;
        double throttleRight = leftY + (leftY * positivePart(rightX)) - triggerThrottle;
        driveTrain.tankDrivePercent(throttleLeft, throttleRight);
    }
}
