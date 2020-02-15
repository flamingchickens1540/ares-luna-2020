package org.team1540.robot2020.commands.hood;

import edu.wpi.first.wpilibj2.command.CommandBase;
import org.team1540.robot2020.utils.ChickenXboxController;

public class HoodManualControl extends CommandBase {
    private Hood hood;
    private ChickenXboxController.Axis joystickAxis;

    public HoodManualControl(Hood hood, ChickenXboxController.Axis joystickAxis) {
        this.hood = hood;
        this.joystickAxis = joystickAxis;
        addRequirements(hood);
    }

    @Override
    public void execute() {
        hood.setPercent(joystickAxis.withDeadzone(0.1).value());
    }
}
