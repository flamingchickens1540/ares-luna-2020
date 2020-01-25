package org.team1540.robot2020.commands.intake;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import org.team1540.robot2020.subsystems.Intake;

public class IntakeOut extends InstantCommand {
    private Intake intake;

    public IntakeOut(Intake intake) {
        this.intake = intake;
    }

    @Override
    public void initialize() {
        intake.setRollerSpeed(-100);
    }
}
