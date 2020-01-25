package org.team1540.robot2020.commands.shooter;

import edu.wpi.first.wpilibj2.command.CommandBase;
import org.team1540.robot2020.subsystems.Shooter;

public class WaitForShooterUpToSpeed extends CommandBase {
    Shooter shooter;

    public WaitForShooterUpToSpeed(Shooter shooter) {
        this.shooter = shooter;
    }

    @Override
    public boolean isFinished() {
        return Math.abs(shooter.getVelocity() - 2000) <= 50;
    }
}
