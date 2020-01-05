package org.team1540.robot2020.commands.drivetrain;

import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Subsystem;
import org.team1540.robot2020.subsystems.DriveTrain;
import org.team1540.rooster.util.ChickenXboxController;
import org.team1540.rooster.util.ControlUtils;

import java.util.HashSet;
import java.util.Set;

public class TankDrive implements Command {
    private DriveTrain driveTrain;
    private ChickenXboxController driver;

    public TankDrive(DriveTrain driveTrain, ChickenXboxController driver) {
        this.driveTrain = driveTrain;
        this.driver = driver;
    }

    @Override
    public void execute() {
        double triggerThrottle = driver.getTriggerAxis(GenericHID.Hand.kRight) - driver.getTriggerAxis(GenericHID.Hand.kLeft);
        driveTrain.setThrottle(
                ControlUtils.deadzone(driver.getRectifiedX(GenericHID.Hand.kLeft), 0.1),
                ControlUtils.deadzone(driver.getRectifiedX(GenericHID.Hand.kRight), 0.1)
        );
    }

    @Override
    public Set<Subsystem> getRequirements() {
        HashSet<Subsystem> requirements = new HashSet<Subsystem>();
        requirements.add(driveTrain);
        return requirements;
    }
}
