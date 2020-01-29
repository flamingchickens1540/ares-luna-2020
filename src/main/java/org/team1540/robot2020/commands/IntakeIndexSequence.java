package org.team1540.robot2020.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;
import org.team1540.robot2020.commands.indexer.MoveBallsUpOne;
import org.team1540.robot2020.subsystems.Indexer;
import org.team1540.robot2020.subsystems.Intake;

public class IntakeIndexSequence extends CommandBase {
    private Intake intake;
    private Indexer indexer;

    public IntakeIndexSequence(Intake intake, Indexer indexer) {
        this.intake = intake;
        this.indexer = indexer;
        addRequirements(indexer);
    }

    @Override
    public void execute() {
        // TODO: should be a command group, you can't have commands run sequentially by doing this

        if (indexer.isFull()) {
            new IntakeOut(intake).schedule();
        } else {
            new IntakeIn(intake).schedule();
            new WaitUntilCommand(indexer::getIndexerStaged).asProxy().schedule();
            new WaitCommand(0.25).asProxy().schedule();
            new MoveBallsUpOne(indexer, 1).asProxy().schedule();
            indexer.ballAdded();
        }
    }
}
