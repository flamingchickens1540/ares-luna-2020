package org.team1540.robot2020.commands.indexer;

import edu.wpi.first.wpilibj2.command.CommandBase;
import org.team1540.robot2020.subsystems.Indexer;

public class MoveBallsToTop extends CommandBase {
    private Indexer indexer;

    public MoveBallsToTop(Indexer indexer) {
        this.indexer = indexer;
    }

    @Override
    public void initialize() {
        // TODO: If the indexer sensor is already triggered, move the balls the other way
        indexer.setPercent(1);
    }

    @Override
    public boolean isFinished() {
        return indexer.getShooterStaged();
    }
}
