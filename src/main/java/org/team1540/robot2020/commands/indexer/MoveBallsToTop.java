package org.team1540.robot2020.commands.indexer;

import edu.wpi.first.wpilibj.controller.PIDController;
import edu.wpi.first.wpilibj2.command.CommandBase;
import org.team1540.robot2020.subsystems.Indexer;

public class MoveBallsToTop extends CommandBase {
    private Indexer indexer;

    private PIDController pidController = new PIDController(1, 0, 0);

    public MoveBallsToTop(Indexer indexer) {
        this.indexer = indexer;

//        0 is height of indexer in inches
//        1 is height of ball in inches
        pidController.setSetpoint(indexer.getEncoderInches() + 0 - (indexer.getBalls() * 1));
    }

    @Override
    public void execute() {
        indexer.setPercent(pidController.calculate(indexer.getEncoderInches()));
    }

    @Override
    public boolean isFinished() {
        return indexer.getTopSensor();
    }
}
