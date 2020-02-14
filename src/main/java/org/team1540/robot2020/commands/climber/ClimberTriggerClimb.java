package org.team1540.robot2020.commands.climber;

import edu.wpi.first.wpilibj2.command.CommandBase;
import org.team1540.robot2020.utils.ChickenXboxController;

public class ClimberTriggerClimb extends CommandBase {
    private Climber climber;
    private ChickenXboxController.Axis trigger;

    public ClimberTriggerClimb(Climber climber, ChickenXboxController.Axis trigger) {
        this.climber = climber;
        this.trigger = trigger;
    }

    @Override
    public void initialize() {

    }

    @Override
    public void execute() {
        climber.setPercent(-trigger.withDeadzone(0.15).value());
    }

    @Override
    public void end(boolean interrupted) {

    }
}
