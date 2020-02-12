package org.team1540.robot2020.commands.climber;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import org.team1540.robot2020.subsystems.Climber;

/**
 * Moves the climber to a goalMeters without checking the ratchet servo
 */
public class MoveClimberToPositionUnsafeMeters extends CommandBase {
    private Climber climber;
    private double goalMeters;
    private Timer ratchetBrokenDetectionTimer;

    public MoveClimberToPositionUnsafeMeters(Climber climber, double goalMeters) {
        this.climber = climber;
        this.goalMeters = goalMeters;
        this.ratchetBrokenDetectionTimer = new Timer();
    }

    @Override
    public void initialize() {
        SmartDashboard.putBoolean("climber/RatchetBrokenAlert", false);
        climber.setPositionMeters(goalMeters);
        ratchetBrokenDetectionTimer.reset();
        ratchetBrokenDetectionTimer.start();
    }

    @Override
    public boolean isFinished() {
        if (climber.getCurrentDraw() > 1 && climber.getVelocityMeters() < 0.1 && ratchetBrokenDetectionTimer.hasPeriodPassed(0.15)) {
            System.out.println("Climber ratchet broken detected --------------------------------");
            SmartDashboard.putBoolean("climber/RatchetBrokenAlert", true);
            climber.stop();
            return true;
        }
        return climber.atPositionMeters(goalMeters, 0.01);
    }
}
