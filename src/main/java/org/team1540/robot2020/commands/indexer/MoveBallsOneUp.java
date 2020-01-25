package org.team1540.robot2020.commands.indexer;

import edu.wpi.first.wpilibj.controller.PIDController;
import edu.wpi.first.wpilibj2.command.CommandBase;
import org.team1540.robot2020.subsystems.Indexer;

public class MoveBallsOneUp extends CommandBase {
    private Indexer indexer;

    private PIDController pidController = new PIDController(1, 0, 0);

    public MoveBallsOneUp(Indexer indexer) {
        this.indexer = indexer;

//        1 is diameter of ball in inches
        pidController.setSetpoint(indexer.getEncoderInches() + 1);
    }

    @Override
    public void execute() {
        indexer.setPercent(pidController.calculate(indexer.getEncoderInches()));
    }

    @Override
    public boolean isFinished() {
        return pidController.atSetpoint();
    }
}
