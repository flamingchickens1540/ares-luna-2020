package org.team1540.robot2020.commands;

import edu.wpi.first.wpilibj.controller.PIDController;
import edu.wpi.first.wpilibj2.command.CommandBase;
import org.team1540.robot2020.subsystems.DriveTrain;
import org.team1540.robot2020.wrappers.Limelight;

public class PointToVisionTarget extends CommandBase {

    DriveTrain driveTrain;
    PIDController pidController;
    Limelight limelight;

    public PointToVisionTarget(DriveTrain driveTrain, Limelight limelight) {
        this.driveTrain = driveTrain;
        this.limelight = limelight;
        addRequirements(driveTrain);
        pidController = new PIDController(1,0,.1);
    }

    @Override
    public void execute() {
        double pidOutput = pidController.calculate(limelight.getTargetAngles().getX(), 0);
        driveTrain.set(-pidOutput,pidOutput);
    }
}
