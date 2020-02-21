package org.team1540.robot2020.commands.climber;

import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.*;
import edu.wpi.first.wpilibj2.command.button.Button;
import org.team1540.robot2020.utils.ChickenXboxController;
import org.team1540.robot2020.utils.InstCommand;

import java.util.Set;

public class ClimberNonSensorControl extends SequentialCommandGroup {

    public ClimberNonSensorControl(Climber climber, ChickenXboxController.Axis axis, Button button) {
        addCommands(
                new InstCommand(()->climber.setRatchet(Climber.RatchetState.DISENGAGED)),
                new WaitCommand(0.35),
                new ClimberMoveToMetersUnsafe(climber, () -> 0.6),
                parallel(
                        new ClimberJoystickControl(climber, axis),
                        new CommandBase() {

                            boolean buttonInitPressed = false;

                            @Override
                            public void execute() {
                                if(buttonInitPressed) {
                                    climber.setRatchet(button.get() ? Climber.RatchetState.DISENGAGED : Climber.RatchetState.ENGAGED);
                                } else {
                                    if(button.get()) buttonInitPressed = true;
                                }
                            }
                        }
                )
        );
    }
}
