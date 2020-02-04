package org.team1540.robot2020.commands.indexer;

import edu.wpi.first.wpilibj2.command.*;
import org.team1540.robot2020.subsystems.Indexer;
import org.team1540.robot2020.utils.InstCommand;

public class BallQueueSequence extends SequentialCommandGroup {
    public BallQueueSequence(Indexer indexer) {
        addRequirements(indexer);
        addCommands(
                new ConditionalCommand(
                        new WaitUntilCommand(() -> !indexer.isFull()),
                        new SequentialCommandGroup(
                                new WaitUntilCommand(indexer::getIndexerStagedSensor),
                                new WaitCommand(0.25),
                                new MoveBallsUpOne(indexer, 1),
                                new InstantCommand(indexer::ballAdded)
                        ),
                        indexer::isFull
                )
        );
    }

    @Override
    public void end(boolean interrupted) {
        if (!interrupted) {
            this.schedule();
        }
    }
}
