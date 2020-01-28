package org.team1540.robot2020.commands.climber;

import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import org.team1540.robot2020.subsystems.Climber;
import org.team1540.robot2020.utils.ChickenXboxController;

public class ClimberJoystickControl extends CommandBase {
    private Climber climber;
    private ChickenXboxController.Axis joystickAxis;
    private JoystickButton ratchetbutton;

    public ClimberJoystickControl(Climber climber, ChickenXboxController.Axis joystickAxis, JoystickButton ratchetbutton) {
        this.climber = climber;
        this.joystickAxis = joystickAxis;
        this.ratchetbutton = ratchetbutton;
        addRequirements(climber);
    }

    @Override
    public void execute() {
        climber.setPercent(joystickAxis.value());
        climber.setRatchet(ratchetbutton.get());
    }
}
