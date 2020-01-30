package org.team1540.robot2020.commands.climber;

import edu.wpi.first.wpilibj.controller.ProfiledPIDController;
import edu.wpi.first.wpilibj.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj2.command.*;
import org.team1540.robot2020.subsystems.Climber;
import org.team1540.robot2020.utils.InstCommand;

public class MoveClimberToPosition extends SequentialCommandGroup {

    public MoveClimberToPosition(Climber climber, double position) {
        addRequirements(climber);
        addCommands(
                new ConditionalCommand(
                        new InstCommand(() -> climber.setRatchet(true)),
                        new InstCommand(),
                        () -> position >= climber.getPosition()
                ),
                new WaitCommand(0.25),
                new FunctionalCommand(
                        () -> climber.setPosition(position),
                        () -> {},
                        (Boolean interrupted) -> climber.setRatchet(false),
                        () -> Math.abs(climber.getPosition() - position) <= 50
                )
        );
    }
}
