package org.team1540.robot2020.commands.climber;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import org.team1540.robot2020.subsystems.Climber;
import org.team1540.robot2020.utils.ChickenXboxController;

public class RatchetServoManualControl extends CommandBase {
    private Climber climber;
    private ChickenXboxController.Axis joystickAxis;
    private double ratchetValue = 0;

    public RatchetServoManualControl(Climber climber, ChickenXboxController.Axis joystickAxis) {
        this.climber = climber;
        this.joystickAxis = joystickAxis;
        addRequirements(climber);
    }

    @Override
    public void execute() {
        SmartDashboard.putNumber("RatchetValue", ratchetValue);
        ratchetValue += joystickAxis.withDeadzone(0.1).value() / 50;
        climber.setRatchet(ratchetValue);
    }
}
