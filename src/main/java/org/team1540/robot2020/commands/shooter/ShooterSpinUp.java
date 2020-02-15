package org.team1540.robot2020.commands.shooter;

import edu.wpi.first.wpilibj.SlewRateLimiter;
import edu.wpi.first.wpilibj2.command.CommandBase;

public class ShooterSpinUp extends CommandBase {
    private final double rpm;
    private Shooter shooter;

    private SlewRateLimiter velocityRateLimiter = new SlewRateLimiter(3000);

    public ShooterSpinUp(Shooter shooter, double rpm) {
        this.shooter = shooter;
        this.rpm = rpm;
        addRequirements(shooter);
    }

    public boolean atTargetVelocity() {
        return Math.abs(shooter.getClosedLoopError()) < 100;
    }

    @Override
    public void initialize() {
        velocityRateLimiter.reset(shooter.getVelocityRPM());
    }

    @Override
    public void execute() {
        shooter.setVelocityRPM(velocityRateLimiter.calculate(rpm));
    }

    @Override
    public boolean isFinished() {
        return Math.abs(shooter.getVelocityRPM() - rpm) < 100;
    }

    @Override
    public void end(boolean interrupted) {
        shooter.disableMotors();
    }

}

// TODO: keep motor spinning while actually shooting
