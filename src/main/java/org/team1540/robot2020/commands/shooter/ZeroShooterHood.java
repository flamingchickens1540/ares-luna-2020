package org.team1540.robot2020.commands.shooter;

import edu.wpi.first.wpilibj2.command.CommandBase;
import org.team1540.robot2020.subsystems.Shooter;

public class ZeroShooterHood extends CommandBase {
    private Shooter shooter;

    public ZeroShooterHood(Shooter shooter) {
        // TODO this needs to require the shooter
        this.shooter = shooter;
    }

    @Override
    public void execute() {
        // TODO setHoodPercent takes a fraction like 0.25 not a number
        shooter.setHoodPercent(25);
    }

    // TODO you currently don't actually stop the hood or zero its position when the command ends

    @Override
    public boolean isFinished() {
        return shooter.isLimitSwitchPressed();
    }
}
