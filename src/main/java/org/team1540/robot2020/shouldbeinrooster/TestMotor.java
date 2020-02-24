package org.team1540.robot2020.shouldbeinrooster;

import edu.wpi.first.wpilibj2.command.CommandBase;
import org.team1540.rooster.util.ChickenXboxController;

import java.util.function.Supplier;

public class TestMotor extends CommandBase {
    private Supplier<GenericMotor> motorSupplier;
    private ChickenXboxController.Axis axis;

    public TestMotor(Supplier<GenericMotor> motorSupplier, ChickenXboxController.Axis axis) {
        this.motorSupplier = motorSupplier;
        this.axis = axis;
    }

    @Override
    public void execute() {
        motorSupplier.get().setPercent(axis.value());
    }

    @Override
    public void end(boolean interrupted) {
        motorSupplier.get().setPercent(0);
    }
}
