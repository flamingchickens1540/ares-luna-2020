package org.team1540.robot2020.commands.climber;

import edu.wpi.first.wpilibj2.command.ConditionalCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import org.team1540.robot2020.utils.InstCommand;

import java.util.function.DoubleSupplier;

public class ClimberMoveToMeters extends SequentialCommandGroup {

    public ClimberMoveToMeters(Climber climber, DoubleSupplier positionMeters) {
        addRequirements(climber);
        addCommands(
                new ConditionalCommand(
                        sequence(
                                new ClimberRatchetOff(climber),
                                new WaitCommand(0.35)
                        ),
                        new InstCommand(),
                        () -> positionMeters.getAsDouble() >= climber.getPositionMeters()
                ),
                new ClimberMoveToMetersUnsafe(climber, positionMeters),
                new ClimberRatchetOn(climber)
        );
    }
}
