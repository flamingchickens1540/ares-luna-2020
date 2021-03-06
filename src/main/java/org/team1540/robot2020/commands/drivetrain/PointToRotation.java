package org.team1540.robot2020.commands.drivetrain;

import edu.wpi.first.wpilibj.geometry.Pose2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import org.team1540.robot2020.LocalizationManager;
import org.team1540.robot2020.utils.ControlUtils;
import org.team1540.robot2020.utils.MiniPID;
import org.team1540.robot2020.utils.ModifiedMiniPID;
import org.team1540.rooster.util.TrigUtils;

public class PointToRotation extends CommandBase {
    private DriveTrain driveTrain;
    private LocalizationManager localizationManager;
    private Pose2d goal;

    private ModifiedMiniPID pointController;
    private double max;
    private double min;
    private double deadzone;
    private double goalAngle;
    private double toleranceAngle;

    public PointToRotation(DriveTrain driveTrain, LocalizationManager localizationManager, Pose2d goal, double toleranceAngle) {
        this.driveTrain = driveTrain;
        this.localizationManager = localizationManager;
        this.goal = goal;
        this.toleranceAngle = toleranceAngle;

        addRequirements(driveTrain);

        SmartDashboard.putNumber("pointToAngle/P", 0.4);
        SmartDashboard.putNumber("pointToAngle/I", 0);
        SmartDashboard.putNumber("pointToAngle/D", 2);
        SmartDashboard.putNumber("pointToAngle/max", 1);
        SmartDashboard.putNumber("pointToAngle/min", 0.01);
        SmartDashboard.putNumber("pointToAngle/deadzone", 0);

        pointController = new ModifiedMiniPID(0, 0, 0);
    }

    @Override
    public void initialize() {
        double p = SmartDashboard.getNumber("pointToAngle/P", 0);
        double i = SmartDashboard.getNumber("pointToAngle/I", 0);
        double d = SmartDashboard.getNumber("pointToAngle/D", 0);
        max = SmartDashboard.getNumber("pointToAngle/max", 0);
        min = SmartDashboard.getNumber("pointToAngle/min", 0);
        deadzone = SmartDashboard.getNumber("pointToAngle/deadzone", 0);
        pointController.setPID(p, i, d);
        goalAngle = goal.getRotation().getRadians();
    }


    @Override
    public void execute() {
        double error = calculateError();
        SmartDashboard.putNumber("pointToAngle/error", error);

        pointController.setC(localizationManager.hasReachedPointGoal() ? 0 : 0.055);

        double rawPIDOutput = pointController.getOutput(error);
        double angleOutput = ControlUtils.allVelocityConstraints(rawPIDOutput, max, min, deadzone);

        driveTrain.setPercent(angleOutput, -angleOutput);
    }

    private double calculateError() {
        return TrigUtils.signedAngleError(goalAngle, localizationManager.getYawRadians());
    }

    @Override
    public boolean isFinished() {
        return Math.abs(calculateError()) < toleranceAngle;
    }
}
