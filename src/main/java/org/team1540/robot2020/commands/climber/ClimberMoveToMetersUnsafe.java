package org.team1540.robot2020.commands.climber;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;

import java.util.function.DoubleSupplier;

/**
 * Moves the climber to a goalMeters without checking the ratchet servo
 */
public class ClimberMoveToMetersUnsafe extends CommandBase {
    private Climber climber;
    private DoubleSupplier goalMetersSupplier;
    private double goalMeters;
    private Timer ratchetBrokenDetectionTimer;

    public ClimberMoveToMetersUnsafe(Climber climber, DoubleSupplier goalMeters) {
        this.climber = climber;
        this.goalMetersSupplier = goalMeters;
        this.ratchetBrokenDetectionTimer = new Timer();
        addRequirements(climber);
    }

    @Override
    public void initialize() {
        SmartDashboard.putBoolean("climber/RatchetBrokenAlert", false);
        goalMeters = goalMetersSupplier.getAsDouble();
        climber.setPositionMeters(goalMeters);
        ratchetBrokenDetectionTimer.reset();
        ratchetBrokenDetectionTimer.start();
    }

    @Override
    public boolean isFinished() {
        if (climber.getCurrent() > 40 && climber.getVelocityMeters() < 0.1 && ratchetBrokenDetectionTimer.hasPeriodPassed(0.25)) {
            System.out.println("Climber ratchet broken detected --------------------------------");
            SmartDashboard.putBoolean("climber/RatchetBrokenAlert", true);
            climber.stop();
            return true;
        }
        return climber.atPositionMeters(goalMeters, 0.01) && Math.abs(climber.getVelocityMeters()) < 0.01;
    }
}
