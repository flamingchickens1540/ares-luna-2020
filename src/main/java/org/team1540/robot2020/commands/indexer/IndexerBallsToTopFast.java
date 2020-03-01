package org.team1540.robot2020.commands.indexer;

import edu.wpi.first.wpilibj2.command.CommandBase;

public class IndexerBallsToTopFast extends CommandBase {
    private Indexer indexer;
    private double slowSpeed;
    private double fastSpeed;

    public IndexerBallsToTopFast(Indexer indexer, double slowSpeed, double fastSpeed) {
        this.indexer = indexer;
        this.slowSpeed = slowSpeed;
        this.fastSpeed = fastSpeed;
        addRequirements(indexer);
    }

    @Override
    public void initialize() {
        indexer.getShooterStagedSensor();
    }

    @Override
    public void execute() {
        if (indexer.getShooterStagedSensor()) return;
        if (indexer.getAlmostShooterStagedSensor()) {
            indexer.setPercent(slowSpeed);
        } else {
            indexer.setPercent(fastSpeed);
        }
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
