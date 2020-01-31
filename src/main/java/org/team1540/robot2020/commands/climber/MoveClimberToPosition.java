package org.team1540.robot2020.commands.climber;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import org.team1540.robot2020.subsystems.Climber;
import org.team1540.robot2020.utils.InstCommand;

public class MoveClimberToPosition extends SequentialCommandGroup {

    public MoveClimberToPosition(Climber climber, double position) {
        addRequirements(climber);
        addCommands(
                new InstCommand(() -> {
                    if (position >= climber.getPosition()) {
                        climber.setRatchet(true);
                    }
                }),
                new WaitCommand(0.25),
                new MoveClimberToPositionUnsafe(climber, position)
        );
    }
}
