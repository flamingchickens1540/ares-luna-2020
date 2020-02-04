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
        if (!shooter.isLimitSwitchPressed()) {
            shooter.setHoodPercent(HOOD_SPEED_PERCENT);
        } else {
            _isFinished = true;
        }

        SmartDashboard.putBoolean("shooter/limit_switch", shooter.isLimitSwitchPressed());
        SmartDashboard.putNumber("shooter/hood_percent", shooter.getHoodPercent());
    }

    @Override
    public boolean isFinished() {
        return _isFinished;
    }
}
