package org.team1540.robot2020.commands.indexer;

import edu.wpi.first.wpilibj.controller.PIDController;
import edu.wpi.first.wpilibj2.command.CommandBase;
import org.team1540.robot2020.subsystems.Indexer;

public class MoveBallsToTop extends CommandBase {
    private Indexer indexer;

    public MoveBallsToTop(Indexer indexer) {
        this.indexer = indexer;
    }

    @Override
    public void initialize() {
        indexer.setPercent(100);
    }

    @Override
    public boolean isFinished() {
        return indexer.getTopSensor();
    }
}
