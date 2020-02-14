package org.team1540.robot2020.commands.indexer;

import edu.wpi.first.wpilibj2.command.CommandBase;

import java.util.function.DoubleSupplier;

public class IndexerPercentToPosition extends CommandBase {
    private final double firstIndexingSpeed;
    private Indexer indexer;
    private DoubleSupplier goalSupplierMeters;
    private double goalMeters;
    private boolean forwards;

    public IndexerPercentToPosition(Indexer indexer, DoubleSupplier goalSupplierMeters, double firstIndexingSpeed) {
        this.indexer = indexer;
        this.goalSupplierMeters = goalSupplierMeters;
        this.firstIndexingSpeed = firstIndexingSpeed;
        addRequirements(indexer);
    }

    @Override
    public void initialize() {
        goalMeters = goalSupplierMeters.getAsDouble();
        forwards = goalMeters > indexer.getPositionMeters();
        indexer.setPercent((forwards ? 1 : -1) * firstIndexingSpeed);
    }

    @Override
    public boolean isFinished() {
        if (forwards)
            return indexer.getPositionMeters() > goalMeters;
        else
            return indexer.getPositionMeters() < goalMeters;
    }

    @Override
    public void end(boolean interrupted) {
        indexer.setPercent(0);
    }
}
