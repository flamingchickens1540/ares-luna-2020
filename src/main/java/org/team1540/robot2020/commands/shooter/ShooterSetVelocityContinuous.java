package org.team1540.robot2020.commands.shooter;

import edu.wpi.first.wpilibj.SlewRateLimiter;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import org.team1540.robot2020.utils.ControlUtils;

import java.util.function.DoubleSupplier;

public class ShooterSetVelocityContinuous extends CommandBase {
    public static final int HIGH_RPM_kD = 30;
    public static final int LOW_RPM_kD = 10;
    private DoubleSupplier targetRPMSupplier;
    private Shooter shooter;
    private Timer kdTimer;

    private SlewRateLimiter velocityRateLimiter = new SlewRateLimiter(3000);

    public ShooterSetVelocityContinuous(Shooter shooter, DoubleSupplier targetRPMSupplier) {
        this.shooter = shooter;
        this.targetRPMSupplier = targetRPMSupplier;
        addRequirements(shooter);
        kdTimer = new Timer();
    }

    @Override
    public void initialize() {
        velocityRateLimiter.reset(shooter.getVelocityRPM());
        shooter.config_kD(HIGH_RPM_kD);
        kdTimer.reset();
        kdTimer.start();
    }

    private void configLowRPM(double targetRPM) {
        double kD = ControlUtils.linearDeadzoneRamp(targetRPM, false, HIGH_RPM_kD, LOW_RPM_kD, 5000, 3000);
        shooter.config_kD(kD);
        SmartDashboard.putNumber("ShooterLineUpSequence/calculatedKd", kD);
    }

    @Override
    public void execute() {
        double targetRPM = this.targetRPMSupplier.getAsDouble();
        if (kdTimer.hasPeriodPassed(0.3)) configLowRPM(targetRPM);
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
