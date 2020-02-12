package org.team1540.robot2020.commands.climber;

import edu.wpi.first.wpilibj2.command.ConditionalCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import org.team1540.robot2020.subsystems.Climber;
import org.team1540.robot2020.utils.InstCommand;

public class MoveClimberToPositionMeters extends SequentialCommandGroup {

    public MoveClimberToPositionMeters(Climber climber, double positionMeters) {
        addRequirements(climber);
        addCommands(
                new ConditionalCommand(
                        sequence(
                                new RatchetOff(climber),
                                new WaitCommand(0.25)
                        ),
                        new InstCommand(),
                        () -> positionMeters >= climber.getPositionMeters()
                ),
                new MoveClimberToPositionUnsafeMeters(climber, positionMeters),
                new RatchetOff(climber)
        );
    }
}
