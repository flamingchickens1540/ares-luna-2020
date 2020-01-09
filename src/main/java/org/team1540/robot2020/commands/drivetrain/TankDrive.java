package org.team1540.robot2020.commands.drivetrain;

import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.GenericHID.Hand;
import edu.wpi.first.wpilibj2.command.CommandBase;
import org.team1540.robot2020.subsystems.DriveTrain;
import org.team1540.rooster.util.ChickenXboxController;
import org.team1540.rooster.util.ControlUtils;

public class TankDrive extends CommandBase {
    private DriveTrain driveTrain;
    private ChickenXboxController driver;
    private final Limelight limelight;
    private MiniPID pointController = new MiniPID(0, 0, 0);

    public TankDrive(DriveTrain driveTrain, ChickenXboxController driver, Limelight limelight) {
        this.driveTrain = driveTrain;
        this.driver = driver;

        addRequirements(driveTrain);
    }

    @Override
    public void execute() {
        double triggerThrottle = ControlUtils.deadzone(driver.getTriggerAxis(GenericHID.Hand.kRight) - driver.getTriggerAxis(GenericHID.Hand.kLeft), 0.1);
        double leftSpeed = ControlUtils.deadzone(driver.getRectifiedX(Hand.kLeft), 0.1) + triggerThrottle;
        double rightSpeed = ControlUtils.deadzone(driver.getRectifiedX(Hand.kRight), 0.1) + triggerThrottle;
        driveTrain.tankDriveVolts(leftSpeed * 12, rightSpeed * 12);
    }
}
