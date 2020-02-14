package org.team1540.robot2020.commands.controlpanel;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import org.team1540.robot2020.utils.ChickenXboxController;

public class ControlPanelServoManualControl extends CommandBase {
    private ControlPanel controlPanel;
    private ChickenXboxController.Axis joystickAxis;
    private double ratchetValue = 0;

    public ControlPanelServoManualControl(ControlPanel controlPanel, ChickenXboxController.Axis joystickAxis) {
        this.controlPanel = controlPanel;
        this.joystickAxis = joystickAxis;
        addRequirements(controlPanel);
    }

    @Override
    public void execute() {
        SmartDashboard.putNumber("ControlPanelServoValue", ratchetValue);
        ratchetValue += joystickAxis.withDeadzone(0.15).value() / 50;
        controlPanel.setArmServo(ratchetValue);
    }
}
