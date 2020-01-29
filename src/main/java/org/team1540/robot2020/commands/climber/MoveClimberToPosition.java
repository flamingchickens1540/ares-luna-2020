package org.team1540.robot2020.commands.climber;

import edu.wpi.first.wpilibj.controller.ProfiledPIDController;
import edu.wpi.first.wpilibj.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj2.command.CommandBase;
import org.team1540.robot2020.subsystems.Climber;

public class MoveClimberToPosition extends CommandBase {
    private Climber climber;
    private ProfiledPIDController pidController = new ProfiledPIDController(1, 0, 0,
            new TrapezoidProfile.Constraints(10, 20));
    private double position;

    public MoveClimberToPosition(Climber climber, double position) {
        this.climber = climber;
        this.position = position;
        addRequirements(climber);
        pidController.setGoal(position);
    }

    @Override
    public void initialize() {
//        20 is half the climber's height
        // TODO: Shouldn't the climber always disable the ratchet before moving upwards? Like if the new pos is greater than the old pos?
        if (position >= 20) {
            // TODO: Wait for a sec after disabling the ratchet to start moving
            climber.setRatchet(false);
        }
    }

    @Override
    public void execute() {
        climber.setPercent(pidController.calculate(climber.getPosition()));
    }

    @Override
    public void end(boolean interrupted) {
        climber.setRatchet(true);
    }
}
