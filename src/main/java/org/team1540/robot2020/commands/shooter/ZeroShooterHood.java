package org.team1540.robot2020.commands.shooter;

import edu.wpi.first.wpilibj2.command.CommandBase;
import org.team1540.robot2020.subsystems.Shooter;

public class ZeroShooterHood extends CommandBase {
    private Shooter shooter;

    public ZeroShooterHood(Shooter shooter) {
        // TODO this needs to require the shooter
        this.shooter = shooter;
        addRequirements(shooter);
    }

    @Override
    public void execute() {
        shooter.setHoodPercent(0.25);
    }

    @Override
    public boolean isFinished() {
        return shooter.isLimitSwitchPressed();
    }

    @Override
    public void end(boolean interrupted) {
        shooter.setHoodPercent(0);
        shooter.setHoodPosition(0);
    }
}
