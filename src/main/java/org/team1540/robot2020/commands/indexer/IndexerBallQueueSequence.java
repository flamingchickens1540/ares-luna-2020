package org.team1540.robot2020.commands.indexer;

import edu.wpi.first.wpilibj2.command.*;
import org.team1540.robot2020.commands.funnel.Funnel;
import org.team1540.robot2020.commands.funnel.RunFunnel;

public class IndexerBallQueueSequence extends SequentialCommandGroup {
    private boolean endFlag;

    public IndexerBallQueueSequence(Indexer indexer, Funnel funnel) {
        addRequirements(indexer, funnel);
        addCommands(
                new ConditionalCommand(new PrintCommand("Indexer not running because top sensor is tripped"),
                        race(
                                new WaitUntilCommand(indexer::getShooterStagedSensor)
                                        .andThen(new ScheduleCommand(
                                                new IndexerBallsToTop(indexer, Indexer.secondIndexingSpeed)
                                                        .andThen(new StartEndCommand(() -> indexer.setPercent(Indexer.secondIndexingSpeed), () -> indexer.setPercent(0)).withTimeout(0.1))
                                        )),
                                new InstantCommand(() -> indexer.setPercent(0))
                                        .andThen(new WaitForDebouncedIndexerSensor(indexer, 0.25))
                                        .andThen(new RunCommand(() -> indexer.setPercent(Indexer.firstIndexingSpeed))
                                                .withInterrupt(() -> !indexer.getIndexerStagedSensor())
                                                .andThen(() -> indexer.setPercent(0))
                                                .andThen(indexer::ballAdded))
                                        .andThen(new InstantCommand(() -> new IndexerBallQueueSequence(indexer, funnel).schedule())) // not a schedule command to avoid stack overflow
                                        .raceWith(new RunFunnel(funnel, indexer))
                        ), indexer::getShooterStagedSensor)
        );
    }
}
