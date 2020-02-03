package org.team1540.robot2020.commands.intake;

import edu.wpi.first.wpilibj2.command.CommandBase;
import org.team1540.robot2020.subsystems.Indexer;
import org.team1540.robot2020.subsystems.Intake;

public class IntakeWhileNotFull extends CommandBase {
    private Intake intake;
    private Indexer indexer;

    public IntakeWhileNotFull(Intake intake, Indexer indexer) {
        this.intake = intake;
        this.indexer = indexer;
        addRequirements(intake);
    }

    @Override
    public void initialize() {
        // TODO this only run on initialize, did you mean to put this in periodic?
        intake.setFunnelAndRollerPercent(!indexer.isFull());
    }
}