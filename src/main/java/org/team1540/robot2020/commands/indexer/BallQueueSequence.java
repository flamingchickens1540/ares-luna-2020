package org.team1540.robot2020.commands.indexer;

import edu.wpi.first.wpilibj2.command.ConditionalCommand;
import edu.wpi.first.wpilibj2.command.FunctionalCommand;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import org.team1540.robot2020.subsystems.Funnel;
import org.team1540.robot2020.subsystems.Indexer;

public class BallQueueSequence extends SequentialCommandGroup {

    public BallQueueSequence(Indexer indexer, Funnel funnel) {        // TODO: ATTN this doesn't work
        addRequirements(indexer, funnel);
        addCommands(
                race(
                        sequence(
                                new IndexerStagedForAWhile(indexer, 0.3),
                                race(
                                        sequence(new FunctionalCommand(
                                                        () -> indexer.setPercent(0.3),
                                                        () -> {
                                                        },
                                                        (interrupted) -> indexer.bottomOfBottomBallMeters = indexer.getPositionMeters(),
                                                        () -> !indexer.getIndexerStagedSensor(),
                                                        indexer),
                                                new IndexerMoveToPosition(indexer, () -> indexer.bottomOfBottomBallMeters + 0.02, 0.3, 0.001),
                                                new InstantCommand(indexer::ballAdded)),
                                        new FunctionalCommand(
                                                () -> funnel.setPercent(0.5, 0.1),
                                                () -> {
                                                },
                                                (interrupted) -> funnel.stop(),
                                                () -> false,
                                                funnel
                                        )
                                )
                        ),
                        new FunctionalCommand(
                                () -> {
                                },
                                () -> {
                                },
                                (interrupted) -> endFlag = !interrupted,
                                indexer::getShooterStagedSensor
                        )
                ),
                new ConditionalCommand(
                        new StageBallsForShooting(indexer),
                        new InstantCommand(),
                        () -> endFlag)
        );
    }

    private boolean endFlag;

    @Override
    public void initialize() {
        endFlag = false;
        super.initialize();
    }

    @Override
    public void end(boolean interrupted) {
        super.end(interrupted);
        if (!interrupted && !endFlag) {
            this.schedule();
        }
    }
}
