package org.team1540.robot2020.commands.climber;

import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import org.team1540.robot2020.subsystems.Climber;
import org.team1540.robot2020.utils.ChickenXboxController;

public class ClimberManualControl extends CommandBase {
    private Climber climber;
    private ChickenXboxController.Axis joystickAxis;
    private JoystickButton ratchetButton;

    public ClimberManualControl(Climber climber, ChickenXboxController.Axis joystickAxis, JoystickButton ratchetButton) {
        this.climber = climber;
        this.joystickAxis = joystickAxis;
        this.ratchetButton = ratchetButton;
        addRequirements(climber);
    }

    @Override
    public void execute() {
        climber.setPercent(joystickAxis.withDeadzone(0.15).value());
        climber.setRatchet(ratchetButton.get() ? Climber.RatchetState.ON : Climber.RatchetState.OFF);
    }
}
