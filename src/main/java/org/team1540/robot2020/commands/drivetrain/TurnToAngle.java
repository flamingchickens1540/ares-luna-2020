package org.team1540.robot2020.commands.drivetrain;

import edu.wpi.first.wpilibj.controller.ProfiledPIDController;
import edu.wpi.first.wpilibj.geometry.Pose2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj2.command.CommandBase;
import org.team1540.robot2020.LocalizationManager;
import org.team1540.robot2020.utils.RamseteUtils;
import org.team1540.rooster.util.TrigUtils;

import java.util.function.Supplier;

public class TurnToAngle extends CommandBase {
    private DriveTrain driveTrain;
    private LocalizationManager localizationManager;
    private Supplier<Pose2d> offsetSupplier;
    private Pose2d goal;
    private ProfiledPIDController pidController;
    private double goalRadians;

    public TurnToAngle(DriveTrain driveTrain, LocalizationManager localizationManager, Supplier<Pose2d> offsetSupplier, Pose2d goal) {
        this.driveTrain = driveTrain;
        this.localizationManager = localizationManager;
        this.offsetSupplier = offsetSupplier;
        this.goal = goal;
        addRequirements(driveTrain);

        SmartDashboard.putNumber("turnToAngle/P", 0.2);
        SmartDashboard.putNumber("turnToAngle/I", 0);
        SmartDashboard.putNumber("turnToAngle/D", 0);

        SmartDashboard.putNumber("turnToAngle/maxVelocity", 0);
        SmartDashboard.putNumber("turnToAngle/maxAccel", 0);
        SmartDashboard.putNumber("turnToAngle/tolerance", 0);
    }

    @Override
    public void initialize() {
        Pose2d translatePose = RamseteUtils.translatePose(offsetSupplier.get(), goal);
        SmartDashboard.putNumber("turnToAngle/offset", offsetSupplier.get().getRotation().getDegrees());
        SmartDashboard.putNumber("turnToAngle/newGoal", translatePose.getRotation().getDegrees());
        double p = SmartDashboard.getNumber("turnToAngle/P", 0);
        double i = SmartDashboard.getNumber("turnToAngle/I", 0);
        double d = SmartDashboard.getNumber("turnToAngle/D", 0);
        double maxVelocity = SmartDashboard.getNumber("turnToAngle/maxVelocity", 0);
        double maxAccel = SmartDashboard.getNumber("turnToAngle/maxAccel", 0);
        double tolerance = SmartDashboard.getNumber("turnToAngle/tolerance", 0);
        pidController = new ProfiledPIDController(p, i, d,
                new TrapezoidProfile.Constraints(maxVelocity, maxAccel));
        goalRadians = translatePose.getRotation().getRadians();
        pidController.setGoal(0);
        pidController.setTolerance(Math.toDegrees(tolerance));
    }

    @Override
    public void execute() {
        double output = pidController.calculate(TrigUtils.signedAngleError(goalRadians, localizationManager.getYawRadians()));
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