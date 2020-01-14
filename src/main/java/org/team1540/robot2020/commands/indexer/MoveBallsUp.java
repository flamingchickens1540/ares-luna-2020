package org.team1540.robot2020.commands.indexer;

import edu.wpi.first.wpilibj2.command.CommandBase;
import org.team1540.robot2020.subsystems.Indexer;

public class MoveBallsUp extends CommandBase {
    private Indexer indexer;

    public MoveBallsUp(Indexer indexer) {
        this.indexer = indexer;
    }

    @Override
    public void initialize() {
        indexer.setSpeed(100);
    }

    @Override
    public boolean isFinished() {
        return indexer.getTopSensor();
    }
}
