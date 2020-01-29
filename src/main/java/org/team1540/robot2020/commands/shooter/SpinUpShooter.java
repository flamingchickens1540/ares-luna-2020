package org.team1540.robot2020.commands.shooter;

import edu.wpi.first.wpilibj2.command.CommandBase;
import org.team1540.robot2020.subsystems.Shooter;

public class SpinUpShooter extends CommandBase {

    private final int targetVelocityTicksPerDecisecond;
    private Shooter shooter;

    public SpinUpShooter(Shooter shooter, int targetVelocityTicksPerDecisecond) {
        this.shooter = shooter;
        this.targetVelocityTicksPerDecisecond = targetVelocityTicksPerDecisecond;
    }

    @Override
    public void initialize() {
        shooter.setVelocityTicksPerDecisecond(targetVelocityTicksPerDecisecond);
    }

    @Override
    public boolean isFinished() {
        // TODO: the 50 should be a tuning value
        return Math.abs(shooter.getVelocityTicksPerDecisecond() - targetVelocityTicksPerDecisecond) <= 50;
    }
}

// TODO: keep motor spinning while actually shooting