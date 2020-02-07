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
        // TODO you're going to want to wait a little bit of time before allowing the command to end because of this,
        //  because when it's just starting to move up it will have high current draw and low velocity; we'd only start
        //  being concerned when that's the case for more than about half a second
        if (climber.getCurrentDraw() > Climber.currentThreshold && climber.getVelocity() < Climber.velocityThreshold) {
            // TODO ending this command won't actually stop the motors (they'll still be trying to maintain position).
            //  Normally this would be desired behavior (we want the elevator to stay there after we move it)
            //  but if we're canceling the command because the elevator is stuck we *probably* don't want the motors to
            //  keep moving.
            //  so yeah, figure out a way to do that
            return true;
        }
        return climber.atPositionMeters(positionMeters, toleranceMeters);
    }
}
