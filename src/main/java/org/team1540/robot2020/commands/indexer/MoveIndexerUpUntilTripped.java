package org.team1540.robot2020.commands.indexer;

import edu.wpi.first.wpilibj2.command.CommandBase;
import org.team1540.robot2020.subsystems.Indexer;

public class MoveIndexerUpUntilTripped extends CommandBase {
    Indexer indexer;

    public MoveIndexerUpUntilTripped(Indexer indexer) {
        this.indexer = indexer;
    }

    @Override
    public void initialize() {
        indexer.setPercent(1);
    }

    @Override
    public boolean isFinished() {
        return indexer.getTopSensor();
    }
}
