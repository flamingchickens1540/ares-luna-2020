package org.team1540.robot2020.commands.indexer;

import edu.wpi.first.wpilibj2.command.*;
import org.team1540.robot2020.commands.funnel.Funnel;
import org.team1540.robot2020.commands.funnel.FunnelRun;
import org.team1540.robot2020.utils.InstCommand;

public class IndexerBallQueueSequence extends SequentialCommandGroup {
    public IndexerBallQueueSequence(Indexer indexer, Funnel funnel, boolean repeat) {
        addRequirements(indexer, funnel);
        addCommands(
                new ConditionalCommand( // only run if the top sensor isn't tripped
                        new PrintCommand("Indexer not running because top sensor is tripped"),
                        race(
                                new FunnelRun(funnel),
                                sequence(
                                        race(
                                                sequence(
                                                        indexer.commandStop(),
                                                        new WaitForDebouncedIndexerSensor(indexer, 0.25),
                                                        indexer.commandPercent(Indexer.firstIndexingSpeed).withInterrupt(() -> !indexer.getIndexerStagedSensor()),
                                                        new InstantCommand(() -> {
                                                            indexer.ballAdded();
                                                            // not a schedulecommand to avoid stack overflow on init
                                                            if (repeat)
                                                                new IndexerBallQueueSequence(indexer, funnel, true).schedule();
                                                        })
                                                ),
                                                new WaitUntilCommand(indexer::getShooterStagedSensor)
                                        ),
                                        new ConditionalCommand(
                                                sequence(
                                                        new IndexerBallsUpOrDownToTop(indexer, Indexer.secondIndexingSpeed),
                                                        new IndexerBallsUpToTop(indexer, Indexer.secondIndexingSpeed)
                                                ),
                                                new InstCommand(), indexer::getShooterStagedSensor
                                        )
                                )
                        ),
                        indexer::getShooterStagedSensor
                )
        );
    }
}
