package org.team1540.robot2020.commands.climber;

import edu.wpi.first.wpilibj2.command.ConditionalCommand;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import org.team1540.robot2020.subsystems.Climber;
import org.team1540.robot2020.utils.InstCommand;

public class MoveClimberToPosition extends SequentialCommandGroup {

    public MoveClimberToPosition(Climber climber, double positionMeters) {
        addRequirements(climber);
        addCommands(
                new ConditionalCommand(
                        new RatchetOff(climber).andThen(new WaitCommand((0.25))),
                        new InstCommand(),
                        () -> positionMeters >= climber.getPositionMeters()
                ),
                new MoveClimberToPositionUnsafe(climber, positionMeters, 0.01),
                new InstCommand(() -> climber.setRatchet(true))
        );
    }
}
