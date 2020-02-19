package org.team1540.robot2020.commands;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.BaseMotorController;
import edu.wpi.first.wpilibj2.command.CommandBase;
import org.team1540.rooster.util.ChickenXboxController;

import java.util.function.IntSupplier;

public class TestMotors extends CommandBase {
    private BaseMotorController[] motors;
    private IntSupplier motorISupplier;
    private ChickenXboxController.Axis joystickAxis;

    public TestMotors(BaseMotorController[] motors, IntSupplier motorISupplier, ChickenXboxController.Axis joystickAxis) {
        this.motors = motors;
        this.motorISupplier = motorISupplier;
        this.joystickAxis = joystickAxis;
    }

    @Override
    public void execute() {
        motors[motorISupplier.getAsInt()].set(ControlMode.PercentOutput, joystickAxis.value());
    }
}
