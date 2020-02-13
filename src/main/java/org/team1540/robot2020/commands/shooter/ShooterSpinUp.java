package org.team1540.robot2020.commands.shooter;

import edu.wpi.first.wpilibj.SlewRateLimiter;
import edu.wpi.first.wpilibj2.command.CommandBase;
import org.team1540.robot2020.subsystems.Shooter;

public class ShooterSpinUp extends CommandBase {
    private final double FLYWHEEL_RPM = 5000;
    private final double FLYWHEEL_RPM_TOLERANCE = 100;

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

    //    @Override
//    public boolean isFinished() {
//        return Math.abs(shooter.getFlywheelVelocityRPM() - TARGET_FLYWHEEL_RPM) < TARGET_FLYWHEEL_TOLERANCE;
//    }
}

// TODO: keep motor spinning while actually shooting
