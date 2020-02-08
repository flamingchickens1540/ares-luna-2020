package org.team1540.robot2020.commands.indexer;

import edu.wpi.first.wpilibj2.command.CommandBase;
import org.team1540.robot2020.subsystems.Indexer;

public class MoveBallsUpOne extends CommandBase {
    private Indexer indexer;
    private double balls;
    private double setpoint;

    public MoveBallsUpOne(Indexer indexer, double balls) {
        this.indexer = indexer;
        this.balls = balls;
        addRequirements(indexer);
    }

    @Override
    public void initialize() {
        this.setpoint = indexer.getEncoderMeters() + (balls * Indexer.ballSizeMeters);
        indexer.setPercent(Indexer.firstIndexingSpeed);
    }

    @Override
    public boolean isFinished() {
        return Math.abs(setpoint - indexer.getEncoderMeters()) <= Indexer.ballHeightThresholdMeters;
    }

    @Override
    public void end(boolean interrupted) {
        indexer.setPercent(0);
    }
}
