package org.team1540.robot2020.commands.hood;

import edu.wpi.first.wpilibj.LinearFilter;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;

import java.util.function.DoubleSupplier;

public class HoodSetPositionContinuous extends CommandBase {
    private DoubleSupplier positionSupplier;
    private Hood hood;
    private LinearFilter filter = LinearFilter.movingAverage(5);
    private double lastGoal;

    public HoodSetPositionContinuous(Hood hood, DoubleSupplier positionSupplier) {
        this.hood = hood;
        this.positionSupplier = positionSupplier;
        addRequirements(hood);
    }

    @Override
    public void initialize() {
        filter.reset();
    }


    @Override
    public void execute() {
        lastGoal = filter.calculate(this.positionSupplier.getAsDouble());
        double position = lastGoal;
        hood.setPosition(position);
        SmartDashboard.putNumber("ShooterLineUpSequence/hoodSetpoint", position);
    }


    @Override
    public void end(boolean interrupted) {
        hood.setPercent(0);
        SmartDashboard.putBoolean("ShooterLineUpSequence/isHoodGoal", false);
    }
}
