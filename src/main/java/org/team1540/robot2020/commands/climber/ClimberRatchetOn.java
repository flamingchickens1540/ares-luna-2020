package org.team1540.robot2020.commands.climber;

import org.team1540.robot2020.utils.InstCommand;

public class ClimberRatchetOn extends InstCommand {
    private Climber climber;

    public ClimberRatchetOn(Climber climber) {
        this.climber = climber;
    }

    @Override
    public void initialize() {
        climber.setRatchet(Climber.RatchetState.ENGAGED);
    }
}
