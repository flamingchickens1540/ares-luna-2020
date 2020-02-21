package org.team1540.robot2020.commands.climber;

import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.button.Button;
import org.team1540.robot2020.utils.ChickenXboxController;

public class ClimberSequenceSensor extends SequentialCommandGroup {

    public ClimberSequenceSensor(Climber climber, ChickenXboxController.Axis axis, Button button) {
        addCommands(
                new ClimberMoveToMeters(climber, () -> Climber.HOOK_MIN_LOCATION, Climber.RatchetState.DISENGAGED),
                parallel(
                        new ClimberJoystickControl(climber, axis),
                        sequence(
                                new WaitForDebouncedAboveGround(climber, 1, 0.6),
                                new RunCommand(() -> climber.setRatchet(button.get() ? Climber.RatchetState.DISENGAGED : Climber.RatchetState.ENGAGED))
                        ))
        );
    }
}
