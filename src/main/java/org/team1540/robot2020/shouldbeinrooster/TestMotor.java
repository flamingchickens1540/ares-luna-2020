package org.team1540.robot2020.shouldbeinrooster;

import edu.wpi.first.wpilibj2.command.CommandBase;
import org.team1540.rooster.util.ChickenXboxController;

import java.util.ArrayList;
import java.util.List;
import java.util.function.Supplier;

public class TestMotor extends CommandBase {
    private List<GenericMotor> motors;
    private Supplier<Integer> indexSupplier;
    private ChickenXboxController.Axis axis;

    public TestMotor(List<GenericMotor> motors, Supplier<Integer> indexSupplier, ChickenXboxController.Axis axis) {
        this.motors = motors;
        this.indexSupplier = indexSupplier;
        this.axis = axis;
    }

    @Override
    public void initialize() {
        System.out.println("TestMotor Starting --------------------------------------------------");
    }

    @Override
    public void execute() {
        getTestMotor().setPercent(axis.value());
//        for (GenericMotor motor : getOtherMotors()) {
//            motor.setPercent(0);
//        }
    }

    @Override
    public void end(boolean interrupted) {
        for (GenericMotor motor : motors) {
            motor.setPercent(0);
        }
        System.out.println("TestMotor Ending ----------------------------------------");
    }

    private GenericMotor getTestMotor() {
        return motors.get(indexSupplier.get());
    }

    private List<GenericMotor> getOtherMotors() {
        List<GenericMotor> outputMotors = new ArrayList<>(motors);
        outputMotors.remove(indexSupplier.get());
        return outputMotors;
    }
}
