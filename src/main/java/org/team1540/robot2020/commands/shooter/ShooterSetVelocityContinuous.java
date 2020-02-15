package org.team1540.robot2020.commands.shooter;

import edu.wpi.first.wpilibj.SlewRateLimiter;
import edu.wpi.first.wpilibj2.command.CommandBase;

import java.util.function.DoubleSupplier;

public class ShooterSetVelocityContinuous extends CommandBase {
    private DoubleSupplier targetRPM;
    private Shooter shooter;

    private SlewRateLimiter velocityRateLimiter = new SlewRateLimiter(3000);

    public ShooterSetVelocityContinuous(Shooter shooter, DoubleSupplier targetRPM) {
        this.shooter = shooter;
        this.targetRPM = targetRPM;
        addRequirements(shooter);
    }

    @Override
    public void initialize() {
        velocityRateLimiter.reset(shooter.getVelocityRPM());
    }

    @Override
    public void execute() {
        shooter.setVelocityRPM(velocityRateLimiter.calculate(targetRPM.getAsDouble()));
    }

    public boolean hasReachedGoal() {
        return Math.abs(shooter.getVelocityRPM() - targetRPM.getAsDouble()) < 100;
    }

    @Override
    public boolean isFinished() {
        return false;
    }

    @Override
    public void end(boolean interrupted) {
        shooter.disableMotors();
    }
}
