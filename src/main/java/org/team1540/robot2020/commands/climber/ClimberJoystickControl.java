package org.team1540.robot2020.commands.climber;

import edu.wpi.first.wpilibj2.command.CommandBase;
import org.team1540.robot2020.utils.ChickenXboxController;

public class ClimberJoystickControl extends CommandBase {

    Climber climber;
    ChickenXboxController.Axis axis;
    private static final double maxHeight = 0.75;
    private static final double minHeight = 0.05;

    public ClimberJoystickControl(Climber climber, ChickenXboxController.Axis axis) {
        this.climber = climber;
        this.axis = axis;
    }

    @Override
    public void initialize() {
        climber.configSoftLimitMeters(minHeight, maxHeight);
    }

    @Override
    public void execute() {
        climber.setPercent(axis.withDeadzone(0.12).value());
    }
}
