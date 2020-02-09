package org.team1540.robot2020.commands.climber;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.CommandBase;
import org.team1540.robot2020.subsystems.Climber;

/**
 * Moves the climber to a positionMeters without checking the ratchet servo
 */
public class MoveClimberToPositionUnsafe extends CommandBase {
    private Climber climber;
    private double positionMeters;
    private double toleranceMeters;
    private Timer timer;

    public MoveClimberToPositionUnsafe(Climber climber, double positionMeters, double toleranceMeters) {
        this.climber = climber;
        this.positionMeters = positionMeters;
        this.toleranceMeters = toleranceMeters;
        this.timer = new Timer();
    }

    @Override
    public void initialize() {
        climber.setPositionMeters(positionMeters);
        timer.reset();
        timer.start();
    }

    @Override
    public boolean isFinished() {
        if (climber.getCurrentDraw() > Climber.currentThreshold && climber.getVelocity() < Climber.velocityThreshold && timer.hasPeriodPassed(Climber.timeThreshold)) {
            climber.stop();
            return true;
        }
        return climber.atPositionMeters(positionMeters, toleranceMeters);
    }
}
