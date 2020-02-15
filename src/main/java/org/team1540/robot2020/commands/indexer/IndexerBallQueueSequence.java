package org.team1540.robot2020.commands.indexer;

import edu.wpi.first.wpilibj2.command.*;
import org.team1540.robot2020.commands.funnel.Funnel;
import org.team1540.robot2020.commands.funnel.FunnelRun;

public class IndexerBallQueueSequence extends SequentialCommandGroup {
    public IndexerBallQueueSequence(Indexer indexer, Funnel funnel) {
        addRequirements(indexer, funnel);
        addCommands(
                new ConditionalCommand( // only run if the top sensor isn't tripped
                        new PrintCommand("Indexer not running because top sensor is tripped"),
                        race(
                                sequence( // wait for balls to reach the top of the indexer, then stop grabbing new balls and line things up
                                        new WaitUntilCommand(indexer::getShooterStagedSensor),
                                        // schedulecommand so that it immediately interrupts this command group even if we're moving a ball in from the funnel
                                        new ScheduleCommand(sequence(
                                                new IndexerBallsToTop(indexer, Indexer.secondIndexingSpeed),
                                                indexer.commandPercent(Indexer.secondIndexingSpeed).withTimeout(0.1)
                                        ))
                                ),
                                race(
                                        new FunnelRun(funnel),
                                        sequence(
                                                indexer.commandStop(),
                                                new WaitForDebouncedIndexerSensor(indexer, 0.25),
                                                indexer.commandPercent(Indexer.firstIndexingSpeed).withInterrupt(() -> !indexer.getIndexerStagedSensor()),
                                                new InstantCommand(() -> {
                                                    indexer.ballAdded();
                                                    // not a schedulecommand to avoid stack overflow on init
                                                    new IndexerBallQueueSequence(indexer, funnel).schedule();
                                                })
                                        )
                                )
                        ),
                        indexer::getShooterStagedSensor
                )
        );
    }
}
