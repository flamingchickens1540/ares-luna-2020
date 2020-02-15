package org.team1540.robot2020.commands.shooter;

import edu.wpi.first.wpilibj2.command.CommandBase;
import org.team1540.robot2020.utils.ChickenXboxController;

public class ShooterManualControl extends CommandBase {
    private Shooter shooter;
    private ChickenXboxController.Axis joystickAxis;

    public ShooterManualControl(Shooter shooter, ChickenXboxController.Axis joystickAxis) {
        this.shooter = shooter;
        this.joystickAxis = joystickAxis;
        addRequirements(shooter);
    }

    @Override
    public void execute() {
        shooter.setPercent(joystickAxis.value());
    }
}
