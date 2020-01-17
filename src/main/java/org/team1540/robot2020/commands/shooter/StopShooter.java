package org.team1540.robot2020.commands.shooter;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import org.team1540.robot2020.subsystems.Shooter;

public class StopShooter extends InstantCommand {
    private Shooter shooter;

    public StopShooter(Shooter shooter) {
        this.shooter = shooter;
        addRequirements(shooter);
    }

    @Override
    public void initialize() {
        shooter.setVelocity(0);
    }
}
