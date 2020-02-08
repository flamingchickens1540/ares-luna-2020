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
    public void execute() {
        hood.setPosition(0);
    }

    // TODO you currently don't actually stop the hood or zero its position when the command ends

    @Override
    public boolean isFinished() {
        return hood.isLimitSwitchPressed();
    }
}
