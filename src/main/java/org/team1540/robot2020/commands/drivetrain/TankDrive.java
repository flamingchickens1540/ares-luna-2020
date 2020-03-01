package org.team1540.robot2020.commands.drivetrain;

import edu.wpi.first.wpilibj2.command.CommandBase;
import org.team1540.robot2020.utils.ChickenXboxController;

import static org.team1540.robot2020.utils.ChickenXboxController.XboxAxis.*;

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
        double triggerThrottle = driver.getAxis(RIGHT_TRIG).withDeadzone(0.1).value() - driver.getAxis(LEFT_TRIG).withDeadzone(0.1).value();
        double leftSpeed = driver.getAxis(LEFT_X).withDeadzone(0.1).value() + triggerThrottle;
        double rightSpeed = driver.getAxis(RIGHT_X).withDeadzone(0.1).value() + triggerThrottle;
        driveTrain.setPercent(leftSpeed, rightSpeed);
    }
}
