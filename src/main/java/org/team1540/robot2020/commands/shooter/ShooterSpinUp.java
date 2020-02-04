package org.team1540.robot2020.commands.shooter;

import edu.wpi.first.wpilibj2.command.CommandBase;
import org.team1540.robot2020.subsystems.Shooter;

public class ShooterSpinUp extends CommandBase {

    private static final double shooterSpeedToleranceTicksPerDecisecond = 50;

    private final int targetVelocityTicksPerDecisecond;
    private Shooter shooter;

    public ShooterSpinUp(Shooter shooter, int targetVelocityTicksPerDecisecond) {
        this.shooter = shooter;
        this.targetVelocityTicksPerDecisecond = targetVelocityTicksPerDecisecond;
    }

    @Override
    public void initialize() {
        shooter.setFlywheelVelocityTicksPerDecisecond(targetVelocityTicksPerDecisecond);
    }

    @Override
    public boolean isFinished() {
        return Math.abs(shooter.getFlywheelVelocityTicksPerDecisecond() - targetVelocityTicksPerDecisecond) <= shooterSpeedToleranceTicksPerDecisecond;
    }
}

// TODO: keep motor spinning while actually shooting