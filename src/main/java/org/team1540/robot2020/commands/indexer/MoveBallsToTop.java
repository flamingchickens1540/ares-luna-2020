package org.team1540.robot2020.commands.indexer;

import edu.wpi.first.wpilibj2.command.CommandBase;
import org.team1540.robot2020.subsystems.Indexer;

public class MoveBallsToTop extends CommandBase {
    private Indexer indexer;
    private boolean shooterInitiallyStaged;

    public MoveBallsToTop(Indexer indexer) {
        this.indexer = indexer;
    }

    @Override
    public void initialize() {
        shooterInitiallyStaged = indexer.getShooterStaged();
        if (shooterInitiallyStaged) {
            indexer.setPercent(-1);
        } else {
            indexer.setPercent(1);
        }
    }

    @Override
    public boolean isFinished() {
        return shooterInitiallyStaged != indexer.getShooterStaged();
    }
}
