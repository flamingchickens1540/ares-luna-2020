package org.team1540.robot2020.commands.indexer;

import edu.wpi.first.wpilibj2.command.CommandBase;
import org.team1540.robot2020.subsystems.Indexer;

public class StageBallsForShooting extends CommandBase {
    private Indexer indexer;
    private boolean shooterInitiallyStaged;

    public StageBallsForShooting(Indexer indexer) {
        this.indexer = indexer;
    }

    @Override
    public void initialize() {
        shooterInitiallyStaged = indexer.getShooterStagedSensor();
        if (shooterInitiallyStaged) {
            // todo indexer percent tuning constant
            indexer.setPercent(-1);
        } else {
            indexer.setPercent(1);
        }
    }

    @Override
    public boolean isFinished() {
        return shooterInitiallyStaged != indexer.getShooterStagedSensor();
    }
}