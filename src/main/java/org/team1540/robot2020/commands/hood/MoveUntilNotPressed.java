package org.team1540.robot2020.commands.hood;

import edu.wpi.first.wpilibj2.command.CommandBase;
import org.team1540.robot2020.subsystems.Hood;

public class MoveUntilNotPressed extends CommandBase {
    private final double speed;
    private Hood hood;

    public MoveUntilNotPressed(Hood hood, double speed) {
        this.hood = hood;
        this.speed = speed;
        addRequirements(hood);
    }

    @Override
    public void initialize() {
        hood.setPercent(speed);
    }

    @Override
    public boolean isFinished() {
        return !hood.isLimitSwitchPressed();
    }
}
