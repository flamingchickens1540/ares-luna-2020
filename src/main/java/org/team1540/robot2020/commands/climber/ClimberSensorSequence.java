package org.team1540.robot2020.commands.climber;

import edu.wpi.first.wpilibj.AnalogInput;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import org.team1540.robot2020.utils.ChickenXboxController;
import org.team1540.robot2020.utils.InstCommand;

public class ClimberSensorSequence extends SequentialCommandGroup {

    private final double openHookMeters = 0.5;

    public ClimberSensorSequence(Climber climber, ChickenXboxController.Axis axis) {
        addCommands(
                new InstCommand(()->climber.setRatchet(Climber.RatchetState.DISENGAGED)),
                new ClimberMoveToMeters(climber, () -> openHookMeters),
                new ParallelCommandGroup(
                        new ClimberJoystickControl(climber, axis),
                        new ActivateRatchetWhenLifting(climber)));
    }
}
