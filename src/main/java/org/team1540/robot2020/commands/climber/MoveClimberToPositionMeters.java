package org.team1540.robot2020.commands.climber;

import edu.wpi.first.wpilibj2.command.ConditionalCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import org.team1540.robot2020.subsystems.Climber;
import org.team1540.robot2020.utils.InstCommand;

import java.util.function.DoubleSupplier;

public class MoveClimberToPositionMeters extends SequentialCommandGroup {

    public MoveClimberToPositionMeters(Climber climber, DoubleSupplier positionMeters) {
        addRequirements(climber);
        addCommands(
                new ConditionalCommand(
                        sequence(
                                new RatchetOff(climber),
                                new WaitCommand(0.25)
                        ),
                        new InstCommand(),
                        () -> positionMeters.getAsDouble() >= climber.getPositionMeters()
                ),
                new MoveClimberToPositionUnsafeMeters(climber, positionMeters),
                new RatchetOn(climber)
        );
    }
}
