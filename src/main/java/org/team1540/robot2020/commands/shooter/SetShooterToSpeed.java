package org.team1540.robot2020.commands.shooter;

import edu.wpi.first.wpilibj.controller.PIDController;
import edu.wpi.first.wpilibj2.command.CommandBase;
import org.team1540.robot2020.subsystems.Shooter;

public class SetShooterToSpeed extends CommandBase {
    private Shooter shooter;
    private PIDController controller = new PIDController(1, 0, 0);

    public SetShooterToSpeed(Shooter shooter) {
        this.shooter = shooter;
        addRequirements(shooter);
        controller.setSetpoint(2500);
    }

    @Override
    public void execute() {
        shooter.set(controller.calculate(shooter.getSpeedRPM()));
    }

    @Override
    public boolean isFinished() {
        return false;
    }
}
