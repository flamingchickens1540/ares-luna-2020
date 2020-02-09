package org.team1540.robot2020.commands.funnel;

import edu.wpi.first.wpilibj2.command.CommandBase;
import org.team1540.robot2020.subsystems.Funnel;
import org.team1540.robot2020.utils.ChickenXboxController;

public class FunnelManualControl extends CommandBase {
    private Funnel funnel;
    private ChickenXboxController.Axis joystickAxis;

    public FunnelManualControl(Funnel funnel, ChickenXboxController.Axis joystickAxis) {
        this.funnel = funnel;
        this.joystickAxis = joystickAxis;
        addRequirements(funnel);
    }

    @Override
    public void execute() {
        funnel.setPercent(joystickAxis.value());
    }
}
