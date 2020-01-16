package org.team1540.robot2020.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import java.util.function.BooleanSupplier;

public class Intake extends SubsystemBase {
    private CANSparkMax intakeRoller = new CANSparkMax(0, CANSparkMaxLowLevel.MotorType.kBrushless);

    private BooleanSupplier isFull;
    private BooleanSupplier getStagingAreaSensor;

    public Intake(BooleanSupplier isFull, BooleanSupplier getStagingAreaSensor) {
        this.isFull = isFull;
        this.getStagingAreaSensor = getStagingAreaSensor;
    }

    @Override
    public void periodic() {
//        true = isRobotLiningUp()
        if ((isFull.getAsBoolean() || false) && getStagingAreaSensor.getAsBoolean()) {
            intakeRoller.set(-100);
        } else {
            intakeRoller.set(100);
        }
    }
}
