package org.team1540.robot2020.commands.indexer;

import edu.wpi.first.wpilibj2.command.ConditionalCommand;
import edu.wpi.first.wpilibj2.command.FunctionalCommand;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import org.team1540.robot2020.commands.funnel.FunnelWhileNotFull;
import org.team1540.robot2020.subsystems.Funnel;
import org.team1540.robot2020.subsystems.Indexer;
import org.team1540.robot2020.utils.InstCommand;

public class BallQueueSequence extends SequentialCommandGroup {
    private boolean endFlag;

    public BallQueueSequence(Indexer indexer, Funnel funnel) {
        addRequirements(indexer, funnel);
        addCommands(
                race(
                        sequence(
                                new InstantCommand(() -> indexer.setPercent(0)),
                                new IndexerStagedForAWhile(indexer, 0.1),
                                new FunctionalCommand(
                                        () -> indexer.setPercent(Indexer.firstIndexingSpeed),
                                        () -> {},
                                        (interrupted) -> indexer.setPercent(0),
                                        () -> !indexer.getIndexerStagedSensor()
                                ),
                                new InstCommand(indexer::ballAdded)
//                                new IndexerMoveToPosition(indexer, () -> indexer.ballPositions.get(0) + 0.2, 0.3, 0.001)
                        ),
                        new FunnelWhileNotFull(funnel, indexer),
                        new FunctionalCommand(
                                () -> {},
                                () -> {},
                                (interrupted) -> endFlag = !interrupted,
                                indexer::getShooterStagedSensor
                        )
                ),
                new ConditionalCommand(
                        new IndexerBallsToTop(indexer, Indexer.secondIndexingSpeed),
                        new InstCommand(),
                        () -> endFlag
                )
        );
    }

    @Override
    public void initialize() {
        super.initialize();
        endFlag = false;
    }

    @Override
    public void end(boolean interrupted) {
        super.end(interrupted);
        if (!interrupted && !endFlag) {
            this.schedule();
        }
    }
}
