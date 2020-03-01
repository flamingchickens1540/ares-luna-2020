package org.team1540.robot2020.commands.indexer;

import edu.wpi.first.wpilibj2.command.ConditionalCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import org.team1540.robot2020.utils.InstCommand;

public class IndexerStageBallsForShooting extends SequentialCommandGroup {

    public IndexerStageBallsForShooting(Indexer indexer) {
        addRequirements(indexer);
        addCommands(
                new ConditionalCommand(
                        new InstCommand(),
                        new IndexerBallsUpOrDownToTop(indexer, Indexer.firstIndexingSpeed),
                        indexer::getShooterStagedSensor
                ),
                new IndexerBallsUpOrDownToTop(indexer, Indexer.secondIndexingSpeed)
        );
    }
}
