package org.team1540.robot2020.commands.climber;

import org.team1540.robot2020.subsystems.Climber;
import org.team1540.robot2020.utils.InstCommand;

public class RatchetOff extends InstCommand {
    private Climber climber;

    public RatchetOff(Climber climber) {
        this.climber = climber;
    }

    @Override
    public void initialize() {
        climber.setRatchet(false);
    }
}
