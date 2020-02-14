package org.team1540.robot2020.commands.shooter;

import edu.wpi.first.wpilibj.SlewRateLimiter;
import edu.wpi.first.wpilibj2.command.CommandBase;

public class ShooterSpinUp extends CommandBase {
    private final double FLYWHEEL_RPM = 5000;

    private Shooter shooter;

    private SlewRateLimiter velocityRateLimiter = new SlewRateLimiter(3000);

    public ShooterSpinUp(Shooter shooter) {
        this.shooter = shooter;
        addRequirements(shooter);
    }

    @Override
    public void initialize() {
        velocityRateLimiter.reset(shooter.getVelocityRPM());
    }

    @Override
    public void execute() {
        shooter.setVelocityRPM(velocityRateLimiter.calculate(FLYWHEEL_RPM));
    }

    @Override
    public void end(boolean interrupted) {
        shooter.disableMotors();
    }
}

// TODO: keep motor spinning while actually shooting
