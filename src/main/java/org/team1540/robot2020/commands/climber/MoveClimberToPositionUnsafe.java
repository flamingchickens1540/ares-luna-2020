package org.team1540.robot2020.commands.climber;

import edu.wpi.first.wpilibj2.command.CommandBase;
import org.team1540.robot2020.subsystems.Climber;

/**
 * Moves the climber to a position without checking the ratchet servo
 */
public class MoveClimberToPositionUnsafe extends CommandBase {
    private Climber climber;
    private double position;

    public MoveClimberToPositionUnsafe(Climber climber, double positionMeters) {
        this.climber = climber;
        this.position = positionMeters;
    }

    @Override
    public void initialize() {
        // todo write setPositionMeters
        climber.setPositionMeters(position);
    }

    @Override
    public boolean isFinished() {
        // todo write isAtPositionMeters
        return climber.isAtPositionMeters(position);
    }
}
