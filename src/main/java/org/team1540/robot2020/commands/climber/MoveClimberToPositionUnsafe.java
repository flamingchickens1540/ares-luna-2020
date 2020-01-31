package org.team1540.robot2020.commands.climber;

import edu.wpi.first.wpilibj2.command.CommandBase;
import org.team1540.robot2020.subsystems.Climber;

public class MoveClimberToPositionUnsafe extends CommandBase {
    private Climber climber;
    private double position;

    public MoveClimberToPositionUnsafe(Climber climber, double position) {
        this.climber = climber;
        this.position = position;
    }

    @Override
    public void initialize() {
        climber.setPosition(position);
    }

    @Override
    public void end(boolean interrupted) {
        climber.setRatchet(false);
    }

    @Override
    public boolean isFinished() {
        return climber.atPosition(position);
    }
}
