package org.team1540.robot2020.commands;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;
import org.team1540.robot2020.commands.indexer.MoveBallsUpOne;
import org.team1540.robot2020.subsystems.Indexer;

public class IntakeIndexSequence extends SequentialCommandGroup {
    public IntakeIndexSequence(Indexer indexer) {
        addRequirements(indexer);
        addCommands(
                new WaitUntilCommand(indexer::getIndexerStaged),
                new WaitCommand(0.25),
                new MoveBallsUpOne(indexer, 1),
                new InstantCommand(indexer::ballAdded)
        );
    }
}
