package org.team1540.robot2020.commands.climber;

import edu.wpi.first.wpilibj2.command.ConditionalCommand;
import edu.wpi.first.wpilibj2.command.FunctionalCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import org.team1540.robot2020.subsystems.Climber;
import org.team1540.robot2020.utils.InstCommand;

public class MoveClimberToPosition extends SequentialCommandGroup {

    public MoveClimberToPosition(Climber climber, double position) {
        addRequirements(climber);
        addCommands(
                // TODO: this can be done with an if
                new ConditionalCommand(
                        new InstCommand(() -> climber.setRatchet(true)),
                        new InstCommand(),
                        () -> position >= climber.getPosition()
                ),
                new WaitCommand(0.25),
                // TODO this should be a MoveClimberToPositionUnsafe command with an InstCommand after
                new FunctionalCommand(
                        () -> climber.setPosition(position),
                        () -> {},
                        (Boolean interrupted) -> climber.setRatchet(false),
                        () -> Math.abs(climber.getPosition() - position) <= 50
                )
        );
    }
}
