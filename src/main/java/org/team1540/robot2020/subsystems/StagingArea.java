package org.team1540.robot2020.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import java.util.function.BooleanSupplier;

public class StagingArea extends SubsystemBase {
    private CANSparkMax stagingRollers = new CANSparkMax(0, CANSparkMaxLowLevel.MotorType.kBrushless);
    private DigitalInput stagingSensor = new DigitalInput(0);

    private BooleanSupplier isFull;

    public StagingArea(BooleanSupplier isFull) {
        this.isFull = isFull;

        stagingRollers.set(100);
    }

    public Boolean getStagingAreaSensor() {
        return stagingSensor.get();
    }

    @Override
    public void periodic() {
//        false = isRobotLiningUp()
        if (getStagingAreaSensor() && !false && !isFull.getAsBoolean()) {
//            Move indexer up 1 ball length
        }
    }
}
