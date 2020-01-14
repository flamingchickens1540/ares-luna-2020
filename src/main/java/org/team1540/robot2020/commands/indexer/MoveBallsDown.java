package org.team1540.robot2020.commands.indexer;

import edu.wpi.first.wpilibj2.command.CommandBase;
import org.team1540.robot2020.subsystems.Indexer;

public class MoveBallsDown extends CommandBase {
    private Indexer indexer;

    public MoveBallsDown(Indexer indexer) {
        this.indexer = indexer;
    }

    @Override
    public void initialize() {
        indexer.resetEncoder();
        indexer.setSpeed(-100);
    }

    @Override
    public boolean isFinished() {
//        return Math.abs(indexerHeightInTicks - (ballHeightInTicks * balls) - currentTicks) <= thresholdInTicks
        return Math.abs(1000 - (100 * indexer.getBalls()) - indexer.getEncoderInches()) <= 10;
    }
}
