package org.team1540.robot2020.commands.climber;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import org.team1540.robot2020.utils.ChickenXboxController;

public class ClimberSequence extends SequentialCommandGroup {

    public ClimberSequence(Climber climber, ChickenXboxController.Axis axis) {
        addCommands(
                new ClimberMoveToMeters(climber, () -> 0.72),
                new ClimberTriggerClimb(climber, axis)
        );
    }
}
