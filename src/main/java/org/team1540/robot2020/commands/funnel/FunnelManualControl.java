package org.team1540.robot2020.commands.funnel;

import edu.wpi.first.wpilibj2.command.CommandBase;
import org.team1540.robot2020.subsystems.Funnel;
import org.team1540.robot2020.utils.ChickenXboxController;

public class FunnelManualControl extends CommandBase {
    private Funnel funnel;
    private ChickenXboxController.Axis2D joystickAxis;

    public FunnelManualControl(Funnel funnel, ChickenXboxController.Axis2D joystickAxis) {
        this.funnel = funnel;
        this.joystickAxis = joystickAxis;
        addRequirements(funnel);
    }

    @Override
    public void execute() {
        double forward = joystickAxis.x().value() * .6;
        double sideways = joystickAxis.y().value() * .6;
        funnel.setPercent(forward + sideways, forward - sideways);
    }
}
