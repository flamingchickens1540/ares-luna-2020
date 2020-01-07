package org.team1540.robot2020.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import org.team1540.robot2020.subsystems.DriveTrain;
import org.team1540.rooster.util.ChickenXboxController;

public class TankDrive extends CommandBase {

    DriveTrain driveTrain;
    ChickenXboxController.Axis lAxis;
    ChickenXboxController.Axis rAxis;

    public TankDrive(DriveTrain driveTrain, ChickenXboxController.Axis lAxis,  ChickenXboxController.Axis rAxis) {
        super.addRequirements(driveTrain);
        this.driveTrain = driveTrain;
        this.lAxis = lAxis;
        this.rAxis = rAxis;
    }

    @Override
    public void execute() {
        driveTrain.set(lAxis.value(),rAxis.value());
    }
}
