package org.team1540.robot2020.commands.indexer;

import edu.wpi.first.wpilibj.controller.PIDController;
import edu.wpi.first.wpilibj2.command.CommandBase;
import org.team1540.robot2020.subsystems.Indexer;

public class MoveBallsUpOne extends CommandBase {
    private Indexer indexer;

    // TODO remove PID controller
    private PIDController pidController = new PIDController(1, 0, 0);

    public MoveBallsUpOne(Indexer indexer, double balls) {
        this.indexer = indexer;

        // TODO remove PID controller

        pidController.setSetpoint(indexer.getEncoderInches() + balls);
    }

    @Override
    public void execute() {
        // TODO remove PID controller

        indexer.setPercent(pidController.calculate(indexer.getEncoderInches()));
    }

    @Override
    public boolean isFinished() {
        return pidController.atSetpoint();
    }
}
