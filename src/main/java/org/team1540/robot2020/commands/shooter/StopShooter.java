package org.team1540.robot2020.commands.shooter;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import org.team1540.robot2020.subsystems.Shooter;

public class StopShooter extends InstantCommand {
    private Shooter shooter;

    public StopShooter(Shooter shooter) {
        this.shooter = shooter;
    }

    @Override
    public void initialize() {
        // TODO: Would be better to have a stopshooter method in the shooter class
        // TODO: This should set percent instead, we don't want to PID to a value of zero, it can be in coast mode also (extra energy is nice)
        shooter.setVelocityTicksPerDecisecond(0);
    }
}
