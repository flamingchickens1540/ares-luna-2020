package org.team1540.robot2020.commands.shooter;

import edu.wpi.first.wpilibj2.command.CommandBase;
import org.team1540.robot2020.subsystems.Shooter;

public class ShooterSpinUp extends CommandBase {
    private final double FLYWHEEL_RPM = 5900;
    private final double FLYWHEEL_RPM_TOLERANCE = 100;

    private Shooter shooter;

    public ShooterSpinUp(Shooter shooter) {
        this.shooter = shooter;
        addRequirements(shooter);
    }

    @Override
    public void initialize() {
        shooter.setVelocity(FLYWHEEL_RPM);
    }

//    @Override
//    public boolean isFinished() {
//        return Math.abs(shooter.getFlywheelVelocityRPM() - TARGET_FLYWHEEL_RPM) < TARGET_FLYWHEEL_TOLERANCE;
//    }
}

// TODO: keep motor spinning while actually shooting
