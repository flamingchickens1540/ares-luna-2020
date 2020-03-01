package org.team1540.robot2020.commands.hood;

import edu.wpi.first.wpilibj2.command.CommandBase;

public class HoodMoveUntilPressed extends CommandBase {
    private final double speed;
    private boolean shouldBePressed;
    private Hood hood;

    public HoodMoveUntilPressed(Hood hood, double speed, boolean shouldBePressed) {
        this.hood = hood;
        this.speed = speed;
        this.shouldBePressed = shouldBePressed;
        addRequirements(hood);
    }

    @Override
    public void initialize() {
        hood.setPercent(speed);
    }

    @Override
    public boolean isFinished() {
        return shouldBePressed == hood.isLimitSwitchPressed();
    }

    @Override
    public void end(boolean interrupted) {
        hood.setPercent(0);
    }
}
