package org.team1540.robot2020.commands.indexer;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import org.team1540.robot2020.subsystems.Indexer;

public class MoveBallsUpOne extends CommandBase {
    private Indexer indexer;
    private double balls;
    private double setpoint;

    public MoveBallsUpOne(Indexer indexer, double balls) {
        this.indexer = indexer;
        this.balls = balls;
        addRequirements(indexer);
    }

    @Override
    public void initialize() {
        setpoint = indexer.bottomOfBottomBallMeters + (balls * Indexer.ballSizeMeters);
        SmartDashboard.putNumber("indexer/moveBallsUpOneSetpoint", setpoint);
        indexer.setPercent(Indexer.firstIndexingSpeed);
    }

    @Override
    public boolean isFinished() {
        return setpoint - indexer.getEncoderMeters() <= Indexer.ballHeightThresholdMeters &&
                !indexer.getIndexerStagedSensor();
    }

    @Override
    public void end(boolean interrupted) {
        indexer.setPercent(0);
        indexer.bottomOfBottomBallMeters = indexer.getEncoderMeters();
    }
}
