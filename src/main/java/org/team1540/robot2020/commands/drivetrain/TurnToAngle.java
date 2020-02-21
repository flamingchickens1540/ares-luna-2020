package org.team1540.robot2020.commands.drivetrain;

import edu.wpi.first.wpilibj.controller.ProfiledPIDController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj2.command.CommandBase;
import org.team1540.robot2020.LocalizationManager;

public class TurnToAngle extends CommandBase {
    private DriveTrain driveTrain;
    private LocalizationManager localizationManager;
    private ProfiledPIDController pidController = new ProfiledPIDController(0, 0, 0,
            new TrapezoidProfile.Constraints(Math.PI * 2, Math.PI * 100));

    public TurnToAngle(DriveTrain driveTrain, LocalizationManager localizationManager, double goal) {
        this.driveTrain = driveTrain;
        this.localizationManager = localizationManager;
        addRequirements(driveTrain);
        pidController.enableContinuousInput(0, 359);
        pidController.setGoal(goal);
        pidController.setTolerance(0.04);

        SmartDashboard.putNumber("turnToAngle/P", 0.2);
        SmartDashboard.putNumber("turnToAngle/I", 0);
        SmartDashboard.putNumber("turnToAngle/D", 0);
    }

    @Override
    public void initialize() {
        double p = SmartDashboard.getNumber("turnToAngle/P", 0);
        double i = SmartDashboard.getNumber("turnToAngle/I", 0);
        double d = SmartDashboard.getNumber("turnToAngle/D", 0);
        pidController.setPID(p, i, d);
    }

    @Override
    public void execute() {
        double output = pidController.calculate(localizationManager.getAngleRadians());
        driveTrain.setPercent(-output, output);
    }

    @Override
    public boolean isFinished() {
        return pidController.atGoal();
    }

    @Override
    public void end(boolean interrupted) {
        driveTrain.setPercent(0, 0);
    }
}