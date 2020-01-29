package org.team1540.robot2020.commands.intake;

import edu.wpi.first.wpilibj2.command.CommandBase;
import org.team1540.robot2020.subsystems.Indexer;
import org.team1540.robot2020.subsystems.Intake;

public class RunIntake extends CommandBase {
    private Intake intake;
    private Indexer indexer;

    public RunIntake(Intake intake, Indexer indexer) {
        this.intake = intake;
        this.indexer = indexer;
        addRequirements(intake);
    }

    @Override
    public void initialize() {
        if (indexer.isFull()) {
            intake.setSpeed(-1);
        } else {
            intake.setSpeed(1);
        }
    }
}
