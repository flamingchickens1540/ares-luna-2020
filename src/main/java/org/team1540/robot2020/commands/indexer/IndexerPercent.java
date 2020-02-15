package org.team1540.robot2020.commands.indexer;

import edu.wpi.first.wpilibj2.command.CommandBase;

public class IndexerPercent extends CommandBase {
    private final double percent;
    private Indexer indexer;

    public IndexerPercent(Indexer indexer, Double percent) {
        this.indexer = indexer;
        this.percent = percent;
        addRequirements(indexer);
    }

    @Override
    public void initialize() {
        indexer.setPercent(percent);
    }

    @Override
    public void end(boolean interrupted) {
        indexer.setPercent(0);
    }
}
