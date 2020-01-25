package org.team1540.robot2020.commands;

import edu.wpi.first.wpilibj2.command.*;
import org.team1540.robot2020.commands.indexer.MoveBallsOneUp;
import org.team1540.robot2020.commands.intake.IntakeIn;
import org.team1540.robot2020.commands.intake.IntakeOut;
import org.team1540.robot2020.subsystems.Indexer;
import org.team1540.robot2020.subsystems.Intake;

public class IntakeIndexSequence extends CommandBase {
    private Intake intake;
    private Indexer indexer;

    public IntakeIndexSequence(Intake intake, Indexer indexer) {
        this.intake = intake;
        this.indexer = indexer;
        addRequirements(intake, indexer);
    }

    @Override
    public void execute() {
        if (indexer.isFull()) {
            new IntakeOut(intake).schedule();
        } else {
            new IntakeIn(intake).schedule();
            new WaitUntilCommand(indexer::getStagingSensor).asProxy().schedule();
            new WaitCommand(0.25).asProxy().schedule();
            new MoveBallsOneUp(indexer).asProxy().schedule();
        }
    }
}
