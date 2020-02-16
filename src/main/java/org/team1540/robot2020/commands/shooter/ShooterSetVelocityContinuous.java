package org.team1540.robot2020.commands.shooter;

import edu.wpi.first.wpilibj.SlewRateLimiter;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;

import java.util.function.DoubleSupplier;

public class ShooterSetVelocityContinuous extends CommandBase {
    public static final int LOW_RPM_THRESH = 3000;
    public static final int LOW_RPM_BUFFER = 200;
    public static final int HIGH_RPM_kD = 30;
    public static final int LOW_RPM_kD = 10;
    private DoubleSupplier targetRPMSupplier;
    private Shooter shooter;

    private SlewRateLimiter velocityRateLimiter = new SlewRateLimiter(3000);
    private boolean lowRpmMode;

    public ShooterSetVelocityContinuous(Shooter shooter, DoubleSupplier targetRPMSupplier) {
        this.shooter = shooter;
        this.targetRPMSupplier = targetRPMSupplier;
        addRequirements(shooter);
    }

    @Override
    public void initialize() {
        velocityRateLimiter.reset(shooter.getVelocityRPM());
        lowRpmMode = false;
        shooter.config_kD(HIGH_RPM_kD);
    }

    private void configLowRPM(double targetRPM) {
        if (lowRpmMode && targetRPM > LOW_RPM_THRESH + LOW_RPM_BUFFER) {
            lowRpmMode = false;
            shooter.config_kD(HIGH_RPM_kD);
        } else if (!lowRpmMode && targetRPM < LOW_RPM_THRESH - LOW_RPM_BUFFER) {
            lowRpmMode = true;
            shooter.config_kD(LOW_RPM_kD);
        }
    }

    @Override
    public void execute() {
        double targetRPM = this.targetRPMSupplier.getAsDouble();
        configLowRPM(targetRPM);
        shooter.setVelocityRPM(velocityRateLimiter.calculate(targetRPM));
        SmartDashboard.putNumber("ShooterLineUpSequence/shooterSetpoint", targetRPM);
    }

    public boolean hasReachedGoal() {
        return Math.abs(shooter.getVelocityRPM() - targetRPMSupplier.getAsDouble()) < 100;
    }

    @Override
    public void end(boolean interrupted) {
        shooter.stop();
        shooter.config_kD(HIGH_RPM_kD);
    }
}
