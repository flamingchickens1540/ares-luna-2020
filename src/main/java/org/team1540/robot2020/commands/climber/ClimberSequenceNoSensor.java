package org.team1540.robot2020.commands.climber;

import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;
import edu.wpi.first.wpilibj2.command.button.Button;
import org.team1540.robot2020.utils.ChickenXboxController;

public class ClimberSequenceNoSensor extends SequentialCommandGroup {

    public ClimberSequenceNoSensor(Climber climber, ChickenXboxController.Axis axis, Button button) {
        addCommands(
                new ClimberMoveToMeters(climber, () -> Climber.HOOK_MIN_LOCATION, Climber.RatchetState.DISENGAGED),
                parallel(
                        new ClimberJoystickControl(climber, axis),
                        sequence(
                                new WaitUntilCommand(button::get),
                                new RunCommand(() -> climber.setRatchet(button.get() ? Climber.RatchetState.DISENGAGED : Climber.RatchetState.ENGAGED))
                        )
                )
        );
    }
}
