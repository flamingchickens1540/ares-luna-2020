package org.team1540.robot2020.commands.shooter;

import edu.wpi.first.wpilibj2.command.CommandBase;
import org.team1540.robot2020.subsystems.Hood;

public class ZeroHood extends CommandBase {
    private Hood hood;

    public ZeroHood(Hood hood) {
        this.hood = hood;
        addRequirements(hood);
    }

    @Override
    public void initialize() {
        hood.setPercent(25);
    }

    @Override
    public boolean isFinished() {
        return hood.isLimitSwitchPressed();
    }

    @Override
    public void end(boolean interrupted) {
        if (!interrupted) {
            hood.zero();
            hood.setPercent(0);
        }
    }
}
