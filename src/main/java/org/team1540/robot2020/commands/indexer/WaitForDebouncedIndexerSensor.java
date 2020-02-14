package org.team1540.robot2020.commands.indexer;

import edu.wpi.first.wpilibj2.command.CommandBase;
import org.team1540.robot2020.utils.Timer;

public class WaitForDebouncedIndexerSensor extends CommandBase {

    private Indexer indexer;
    private Timer timer;
    private double period;

    protected WaitForDebouncedIndexerSensor(Indexer indexer, double period) {
        this.period = period;
        this.indexer = indexer;
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
        if (indexer.getIndexerStagedSensor()) {
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
