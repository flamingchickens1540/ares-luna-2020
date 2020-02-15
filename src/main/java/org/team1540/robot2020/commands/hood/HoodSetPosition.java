package org.team1540.robot2020.commands.hood;

import edu.wpi.first.wpilibj2.command.CommandBase;

public class HoodSetPosition extends CommandBase {
    private final double position;
    private Hood hood;

    public HoodSetPosition(Hood hood, double position) {
        this.hood = hood;
        this.position = position;
        addRequirements(hood);
    }

    @Override
    public void initialize() {
        hood.setPosition(this.position);
    }

    @Override
    public boolean isFinished() {
        return Math.abs(this.position - hood.getPosition()) < 0.1;
    }
}
