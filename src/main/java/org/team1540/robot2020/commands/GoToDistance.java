package org.team1540.robot2020.commands;

import edu.wpi.first.wpilibj.controller.PIDController;
import edu.wpi.first.wpilibj2.command.CommandBase;
import org.team1540.robot2020.subsystems.DriveTrain;
import org.team1540.robot2020.wrappers.Limelight;

public class GoToDistance extends CommandBase {

    PIDController pidControllerD;
    PIDController pidControllerR;
    DriveTrain driveTrain;
    Limelight limelight;
    double distance;

    public GoToDistance(DriveTrain driveTrain, Limelight limelight, double distance) {
        this.distance = distance;
        this.driveTrain = driveTrain;
        this.limelight = limelight;
        pidControllerD = new PIDController(0.01,0,0);
        pidControllerR = new PIDController(1,0,.1);
    }

    @Override
    public void execute() {
        double pidOD = pidControllerD.calculate(limelight.getDistanceFromSelectedTarget(),distance);
//        driveTrain.set(-pidOD,-pidOD);
        double pidOR = pidControllerR.calculate(limelight.getTargetAngles().getX(), 0);
        driveTrain.set(-pidOD-pidOR,-pidOD+pidOR);
    }
}
