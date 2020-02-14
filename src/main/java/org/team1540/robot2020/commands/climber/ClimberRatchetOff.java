package org.team1540.robot2020.commands.climber;

import org.team1540.robot2020.utils.InstCommand;

public class ClimberRatchetOff extends InstCommand {
    private Climber climber;

    public ClimberRatchetOff(Climber climber) {
        this.climber = climber;
    }

    @Override
    public void initialize() {
        climber.setRatchet(Climber.RatchetState.DISENGAGED);
    }
}
