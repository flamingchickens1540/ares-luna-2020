package org.team1540.robot2020.commands.climber;

import edu.wpi.first.wpilibj2.command.CommandBase;
import org.team1540.robot2020.subsystems.Climber;

/**
 * Moves the climber to a positionMeters without checking the ratchet servo
 */
public class MoveClimberToPositionUnsafe extends CommandBase {
    private Climber climber;
    private double positionMeters;
    private double toleranceMeters;

    private final double currentThreshold = 1;
    private final double velocityThreshold = 1;

    public MoveClimberToPositionUnsafe(Climber climber, double positionMeters, double toleranceMeters) {
        this.climber = climber;
        this.positionMeters = positionMeters;
        this.toleranceMeters = toleranceMeters;
    }

    @Override
    public void initialize() {
        climber.setPositionMeters(positionMeters);
    }

    @Override
    public boolean isFinished() {
        //CHANGE CURRENT DRAW AND VELOCITY THRESHOLD ONCE TESTING IS COMPLETE
        if(climber.getCurrentDraw()>currentThreshold && climber.getVelocity()<velocityThreshold) return true;
        return climber.atPositionMeters(positionMeters, toleranceMeters);
    }
}
