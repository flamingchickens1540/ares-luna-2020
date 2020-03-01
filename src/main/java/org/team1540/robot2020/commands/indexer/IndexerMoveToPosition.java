package org.team1540.robot2020.commands.indexer;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;

import java.util.function.DoubleSupplier;

public class IndexerMoveToPosition extends CommandBase {
    private DoubleSupplier calculateGoalMeters;
    private final double maxPercent;
    private Indexer indexer;
    private double currentGoal;
    private double tolerance;

    public IndexerMoveToPosition(Indexer indexer, DoubleSupplier calculateGoalMeters, double maxPercent, double tolerance) {
        this.indexer = indexer;
        this.calculateGoalMeters = calculateGoalMeters;
        this.maxPercent = maxPercent;
        this.tolerance = tolerance;
        addRequirements(indexer);

        SmartDashboard.putNumber("indexer/position/p", 0.8);
        SmartDashboard.putNumber("indexer/position/i", 0);
        SmartDashboard.putNumber("indexer/position/d", 10);
    }

    @Override
    public void initialize() {
        double p = SmartDashboard.getNumber("indexer/position/p", 0);
        double i = SmartDashboard.getNumber("indexer/position/i", 0);
        double d = SmartDashboard.getNumber("indexer/position/d", 0);
        indexer.setPID(p, i, d);

        currentGoal = calculateGoalMeters.getAsDouble();
        indexer.setPositionMeters(currentGoal, maxPercent);
    }

    @Override
    public boolean isFinished() {
        return indexer.getCloseLoopErrorMeters() < tolerance && Math.abs(indexer.getPositionMeters() - currentGoal) < tolerance;
    }

    @Override
    public void end(boolean interrupted) {
        indexer.setPercent(0);
    }
}
