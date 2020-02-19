package org.team1540.robot2020.commands.climber;

import edu.wpi.first.wpilibj.AnalogInput;
import edu.wpi.first.wpilibj2.command.CommandBase;

public class ActivateRatchetWhenLifting extends CommandBase {

    Climber climber;
    //TODO: Tune voltage threshold
    final double voltageThreshold = 1;

    public ActivateRatchetWhenLifting(Climber climber) {
        this.climber = climber;
    }

    @Override
    public void execute() {
        if(climber.getHeight() > voltageThreshold) climber.setRatchet(Climber.RatchetState.ENGAGED);
    }
}
