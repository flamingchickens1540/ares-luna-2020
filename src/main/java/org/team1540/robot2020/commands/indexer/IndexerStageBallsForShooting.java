package org.team1540.robot2020.commands.indexer;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.ConditionalCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import org.team1540.robot2020.utils.InstCommand;

public class IndexerStageBallsForShooting extends SequentialCommandGroup {

    public IndexerStageBallsForShooting(Indexer indexer) {
        addRequirements(indexer);
        addCommands(
//                new IndexerMoveToPosition(indexer, () -> (3 - indexer.ballPositions.size()) * Indexer.ballSizeMeters, 0.5, 0.05),
                new ConditionalCommand(
                        new InstCommand(),
                        new IndexerBallsToTop(indexer, Indexer.firstIndexingSpeed),
                        indexer::getShooterStagedSensor
                ),
                new IndexerBallsToTop(indexer, Indexer.secondIndexingSpeed)
        );

        SmartDashboard.putNumber("indexer/stageDistance", 0.82);
    }
}
