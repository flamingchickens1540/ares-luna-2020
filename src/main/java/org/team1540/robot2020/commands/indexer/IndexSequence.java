package org.team1540.robot2020.commands.indexer;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;
import org.team1540.robot2020.subsystems.Indexer;

public class IndexSequence extends SequentialCommandGroup {
    public IndexSequence(Indexer indexer) {
        addRequirements(indexer);
        addCommands(
                // TODO indexer should have a max number of balls to intake
                new WaitUntilCommand(indexer::getIndexerStaged),
                new WaitCommand(0.25),
                new MoveBallsUpOne(indexer, 1),
                new InstantCommand(indexer::ballAdded)
        );
    }

    @Override
    public void end(boolean interrupted) {
        if (!interrupted) {
            // TODO test if this works with Phineas
            this.schedule();
        }
    }
}
