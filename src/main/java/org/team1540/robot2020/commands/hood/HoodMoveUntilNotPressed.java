package org.team1540.robot2020.commands.hood;

import edu.wpi.first.wpilibj2.command.CommandBase;

public class HoodMoveUntilNotPressed extends CommandBase {
    private final double speed;
    private Hood hood;

    public HoodMoveUntilNotPressed(Hood hood, double speed) {
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
