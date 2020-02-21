package org.team1540.robot2020.commands.climber;

import edu.wpi.first.wpilibj2.command.CommandBase;
import org.team1540.robot2020.commands.climber.Climber;
import org.team1540.robot2020.utils.Timer;

public class ActivateRatchetAfterAboveGroundAfterPeriod extends CommandBase {

    private Climber climber;
    private Timer timer;
    private double period;
    private final double heightVoltageThreshold;

    public ActivateRatchetAfterAboveGroundAfterPeriod(Climber climber, double period, double heightThreshold) {
        this.period = period;
        this.climber = climber;
        this.heightVoltageThreshold = heightThreshold;
        timer = new Timer();
    }


    @Override
    public void initialize() {
        timer.reset();
        timer.stop();
    }

    @Override
    public void execute() {

    }

    @Override
    public void end(boolean interrupted) {
    }

    @Override
    public boolean isFinished() {
        if (climber.getGroundSensorVoltage() > heightVoltageThreshold) {
            if (!timer.isRunning()) {
                timer.reset();
                timer.start();
            }
        } else {
            timer.stop();
        }

        return timer.hasPeriodPassed(period);
    }
}
