package org.team1540.robot2020.commands.shooter;

import edu.wpi.first.wpilibj2.command.CommandBase;
import org.team1540.robot2020.subsystems.Shooter;

public class SpinUpShooter extends CommandBase {
    private Shooter shooter;

    public SpinUpShooter(Shooter shooter) {
        this.shooter = shooter;
        addRequirements(shooter);
    }

    @Override
    public void initialize() {
        shooter.setVelocity(2000);
    }

    @Override
    public boolean isFinished() {
        return Math.abs(shooter.getSpeedRPM() - 2000) <= 50;

    }
}
