package org.team1540.robot2020.commands.indexer;

import edu.wpi.first.wpilibj2.command.CommandBase;

public class IndexerBallsUpToTop extends CommandBase {
    private Indexer indexer;
    private double slowSpeed;

    public IndexerBallsUpToTop(Indexer indexer, double slowSpeed) {
        this.indexer = indexer;
        this.slowSpeed = slowSpeed;
        addRequirements(indexer);
    }

    @Override
    public void initialize() {

    }

    @Override
    public void execute() {
        if (indexer.getShooterStagedSensor()) return;
        indexer.setPercent(slowSpeed);
    }

    @Override
    public boolean isFinished() {
        return indexer.getShooterStagedSensor();
    }

    @Override
    public void end(boolean interrupted) {
        indexer.setPercent(0);
    }
}
