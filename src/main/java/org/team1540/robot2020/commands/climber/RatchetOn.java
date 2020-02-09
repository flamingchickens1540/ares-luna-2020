package org.team1540.robot2020.commands.climber;

import org.team1540.robot2020.subsystems.Climber;
import org.team1540.robot2020.utils.InstCommand;

public class RatchetOn extends InstCommand {
    private Climber climber;

    public RatchetOn(Climber climber) {
        this.climber = climber;
    }

    @Override
    public void initialize() {
        climber.setRatchet(false);
    }
}
