package org.team1540.robot2020.commands.intake;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import org.team1540.robot2020.subsystems.Intake;

public class OutsideRollerOut extends InstantCommand {
    private Intake intake;

    public OutsideRollerOut(Intake intake) {
        this.intake = intake;
    }

    @Override
    public void initialize() {
        intake.setOutsideRoller(-100);
    }
}
