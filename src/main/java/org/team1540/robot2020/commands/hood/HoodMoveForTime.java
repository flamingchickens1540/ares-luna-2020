package org.team1540.robot2020.commands.hood;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.CommandBase;

public class HoodMoveForTime extends CommandBase {
    private final double speed; // Percent
    private final double time; // Seconds
    private Timer timer = new Timer();
    private Hood hood;

    public HoodMoveForTime(Hood hood, double speed, double time) {
        this.hood = hood;
        this.speed = speed;
        this.time = time;
        addRequirements(hood);
    }

    @Override
    public void initialize() {
        timer.reset();
        timer.start();
        hood.setPercent(speed);
    }

    @Override
    public boolean isFinished() {
        return timer.hasPeriodPassed(time);
    }
}
