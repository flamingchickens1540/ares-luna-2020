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
                new InstCommand(() -> {
                    // TODO do this with a ConditionalCommand
                    if (positionMeters >= climber.getPositionMeters()) {
                        // TODO create a premade RatchetOff and RatchetOn command class or something, so you don't have
                        //  to type new InstCommand(() -> climber.setRatchet(Climber.RatchetState.ON)) every time
                        climber.setRatchet(Climber.RatchetState.OFF);
                    }
                }),
                // TODO only make it wait when you're actually disabling the ratchet (hint: this involves using a
                //  command group inside the conditional command)
                new WaitCommand(0.25),
                new MoveClimberToPositionUnsafe(climber, positionMeters, 0.01),
                new InstCommand(() -> climber.setRatchet(Climber.RatchetState.ON))
        );
    }
}
