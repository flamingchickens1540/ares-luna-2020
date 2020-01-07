package org.team1540.robot2020.commands;

import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.Subsystem;
import org.team1540.robot2020.subsystems.DriveTrain;
import org.team1540.rooster.util.ChickenXboxController;

public class PointDrive extends CommandBase {

    ChickenXboxController.Axis pointX;
    ChickenXboxController.Axis pointY;
    ChickenXboxController.Axis throttle;
    DriveTrain driveTrain;

    public PointDrive(DriveTrain driveTrain, ChickenXboxController.Axis pointX, ChickenXboxController.Axis pointY, ChickenXboxController.Axis throttle) {
        this.pointX = pointX;
        this.pointY = pointY;
        this.throttle = throttle;
        this.driveTrain = driveTrain;
        super.addRequirements(driveTrain);
    }

}
