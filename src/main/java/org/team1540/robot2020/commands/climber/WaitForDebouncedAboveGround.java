package org.team1540.robot2020.commands.climber;

import edu.wpi.first.wpilibj2.command.CommandBase;
import org.team1540.robot2020.utils.Timer;

public class WaitForDebouncedAboveGround extends CommandBase {

    private Climber climber;
    private Timer timer;
    private double period;
    private final double heightVoltageThreshold;

    public WaitForDebouncedAboveGround(Climber climber, double period, double heightVoltageThreshold) {
        this.period = period;
        this.climber = climber;
        this.heightVoltageThreshold = heightVoltageThreshold;
        timer = new Timer();
    }

    @Override
    public void initialize() {
        timer.reset();
        timer.stop();
    }

    @Override
    public void execute() {
        if (climber.getGroundSensorVoltage() > heightVoltageThreshold) {
            if (!timer.isRunning()) {
                timer.reset();
                timer.start();
            }
        } else {
            timer.stop();
        }
    }

    @Override
    public boolean isFinished() {
        return timer.hasPeriodPassed(period);
    }
}
