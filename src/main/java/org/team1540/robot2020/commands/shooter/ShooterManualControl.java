package org.team1540.robot2020.commands.shooter;

import edu.wpi.first.wpilibj2.command.CommandBase;
import org.team1540.robot2020.subsystems.Shooter;
import org.team1540.robot2020.utils.ChickenXboxController;

public class ShooterManualControl extends CommandBase {
    private Shooter intake;
    private ChickenXboxController.Axis joystickAxis;

    public ShooterManualControl(Shooter shooter, ChickenXboxController.Axis joystickAxis) {
        this.intake = shooter;
        this.joystickAxis = joystickAxis;
        addRequirements(shooter);
    }

    @Override
    public void execute() {
        intake.setPercent(joystickAxis.value());
    }
}
