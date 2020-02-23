package org.team1540.robot2020.commands.shooter;

import edu.wpi.first.wpilibj.SlewRateLimiter;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;

import java.util.function.DoubleSupplier;

public class ShooterSetVelocityContinuous extends CommandBase {
    private DoubleSupplier targetRPMSupplier;
    private Shooter shooter;

    private SlewRateLimiter velocityRateLimiter = new SlewRateLimiter(3000);

    public ShooterSetVelocityContinuous(Shooter shooter, DoubleSupplier targetRPMSupplier) {
        this.shooter = shooter;
        this.targetRPMSupplier = targetRPMSupplier;
        addRequirements(shooter);
    }

    @Override
    public void initialize() {
        velocityRateLimiter.reset(shooter.getVelocityRPM());
    }


    @Override
    public void execute() {
        double targetRPM = this.targetRPMSupplier.getAsDouble();
        shooter.setVelocityRPM(velocityRateLimiter.calculate(targetRPM));
        SmartDashboard.putNumber("ShooterLineUpSequence/shooterSetpoint", targetRPM);
    }

    public boolean hasReachedGoal() {
        return Math.abs(shooter.getVelocityRPM() - targetRPMSupplier.getAsDouble()) < 100;
    }

    @Override
    public void end(boolean interrupted) {
        shooter.stop();
    }
}
