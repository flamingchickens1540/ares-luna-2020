package org.team1540.robot2020.commands.intake;

import edu.wpi.first.wpilibj2.command.CommandBase;
import org.team1540.robot2020.utils.Timer;

public class IntakeRun extends CommandBase {
    private Intake intake;
    private double targetRPM;
    private Timer stallTimer;
    private Timer reverseTimer;

    public IntakeRun(Intake intake, double targetRPM) {
        this.intake = intake;
        this.targetRPM = targetRPM;
        addRequirements(intake);
        stallTimer = new Timer();
        reverseTimer = new Timer();
    }

    @Override
    public void initialize() {
        reverseTimer.stop();
    }

    @Override
    public void execute() {
        intake.setVelocity(targetRPM);

        if (Math.abs(intake.getVelocity()) < Math.abs(targetRPM) * 1 / 2) {
            if (!stallTimer.isRunning()) {
                stallTimer.reset();
                stallTimer.start();
            }
        } else {
            stallTimer.stop();
            stallTimer.reset();
        }

        if (stallTimer.hasElapsed(0.3)) {
            reverseTimer.reset();
            reverseTimer.start();
        }

        if (reverseTimer.isRunning() && !reverseTimer.hasElapsed(0.4)) {
            intake.setVelocity(-4000);
            stallTimer.stop();
            stallTimer.reset();
        } else {
            intake.setVelocity(targetRPM);
        }
    }

    @Override
    public void end(boolean interrupted) {
        intake.setPercent(0);
    }
}
