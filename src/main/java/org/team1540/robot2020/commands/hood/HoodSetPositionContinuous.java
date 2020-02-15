package org.team1540.robot2020.commands.hood;

import edu.wpi.first.wpilibj2.command.CommandBase;

import java.util.function.DoubleSupplier;

public class HoodSetPositionContinuous extends CommandBase {
    private DoubleSupplier position;
    private Hood hood;

    public HoodSetPositionContinuous(Hood hood, DoubleSupplier position) {
        this.hood = hood;
        this.position = position;
        addRequirements(hood);
    }

    @Override
    public void execute() {
        hood.setPosition(this.position.getAsDouble());
    }

    public boolean hasReachedGoal() {
        return Math.abs(this.position.getAsDouble() - hood.getPosition()) < 0.1;
    }

    @Override
    public boolean isFinished() {
        return hasReachedGoal();
    }

    @Override
    public void end(boolean interrupted) {
        hood.setPercent(0);
    }
}
