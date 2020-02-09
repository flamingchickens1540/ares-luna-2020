package org.team1540.robot2020.commands.indexer;

import edu.wpi.first.wpilibj2.command.FunctionalCommand;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;
import org.team1540.robot2020.subsystems.Indexer;

public class BallQueueSequence extends SequentialCommandGroup {

    public BallQueueSequence(Indexer indexer) {        // TODO: ATTN this doesn't work
        addRequirements(indexer);
        addCommands(
                new WaitUntilCommand(indexer::getIndexerStagedSensor),
                new FunctionalCommand(
                        () -> indexer.setPercent(0.3),
                        () -> {
                        },
                        (interrupted) -> indexer.bottomOfBottomBallMeters = indexer.getPositionMeters(),
                        () -> !indexer.getIndexerStagedSensor(),
                        indexer),
                new IndexerMoveToPosition(indexer, () -> indexer.bottomOfBottomBallMeters + 0.01, 0.3, 0.001),
                new InstantCommand(indexer::ballAdded)
        );
    }

    @Override
    public void end(boolean interrupted) {
        if (!interrupted) {
            this.schedule();
        }
    }
}
