package org.team1540.robot2020.commands.intake;

import edu.wpi.first.wpilibj2.command.CommandBase;
import org.team1540.robot2020.subsystems.Intake;
import org.team1540.robot2020.utils.ChickenXboxController;

public class IntakeManualControl extends CommandBase {
    private Intake intake;
    private ChickenXboxController.Axis joystickAxis;

    public IntakeManualControl(Intake intake, ChickenXboxController.Axis joystickAxis) {
        this.intake = intake;
        this.joystickAxis = joystickAxis;
        addRequirements(intake);
    }

    @Override
    public void execute() {
        intake.setPercent(joystickAxis.value());
    }
}
