package org.team1540.robot2020.commands.indexer;

import edu.wpi.first.wpilibj2.command.CommandBase;
import org.team1540.robot2020.subsystems.Indexer;

public class MoveBallsToTop extends CommandBase {
    private Indexer indexer;
    private double speed;
    private boolean shooterInitiallyStaged;

    public MoveBallsToTop(Indexer indexer, double speed) {
        this.indexer = indexer;
        this.speed = speed;
        addRequirements(indexer);
    }

    @Override
    public void initialize() {
        shooterInitiallyStaged = indexer.getShooterStagedSensor();
        if (shooterInitiallyStaged) {
            indexer.setPercent(-speed);
        } else {
            indexer.setPercent(speed);
        }
    }

    @Override
    public boolean isFinished() {
        return shooterInitiallyStaged != indexer.getShooterStagedSensor();
    }

    @Override
    public void end(boolean interrupted) {
        indexer.setPercent(0);
    }
}
