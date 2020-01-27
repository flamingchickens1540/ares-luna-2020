package org.team1540.robot2020.commands.climber;

import edu.wpi.first.wpilibj.controller.PIDController;
import edu.wpi.first.wpilibj2.command.CommandBase;
import org.team1540.robot2020.subsystems.Climber;

public class MoveClimberToPosition extends CommandBase {
    private Climber climber;
    private PIDController pidController = new PIDController(1, 0, 0);

    public MoveClimberToPosition(Climber climber, double position) {
        this.climber = climber;
        addRequirements(climber);
        pidController.setSetpoint(position);
    }

    @Override
    public void execute() {
        climber.setPercent(pidController.calculate(climber.getPosition()));
    }
}
