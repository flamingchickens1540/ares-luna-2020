package org.team1540.robot2020.commands.shooter;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import org.team1540.robot2020.subsystems.Shooter;

public class ZeroShooterHood extends CommandBase {
    private final int HOOD_SPEED_PERCENT = 0;
    private Shooter shooter;
    private boolean _isFinished = false;

    public ZeroShooterHood(Shooter shooter) {
        this.shooter = shooter;
    }

    @Override
    public void execute() {
        // TODO the execute method will stop when the command is over, so you don't need to check the limit switch here
        if (!shooter.isLimitSwitchPressed()) {
            shooter.setHoodPercent(HOOD_SPEED_PERCENT);
        } else {
            _isFinished = true;
        }

        // TODO these should be put in the shooter periodic
        SmartDashboard.putBoolean("shooter/limitSwitch", shooter.isLimitSwitchPressed());
        SmartDashboard.putNumber("shooter/hoodPercent", shooter.getHoodPercent());
    }

    @Override
    public boolean isFinished() {
        // TODO the isFinished method runs periodically so you can just return shooter.isLimitSwitchPressed()
        return _isFinished;
    }
}
