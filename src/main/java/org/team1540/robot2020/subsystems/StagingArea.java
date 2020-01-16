package org.team1540.robot2020.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class StagingArea extends SubsystemBase {
    private CANSparkMax stagingRollers = new CANSparkMax(0, CANSparkMaxLowLevel.MotorType.kBrushless);

    private DigitalInput stagingSensor = new DigitalInput(0);

    public Boolean getStagingAreaSensor() {
        return stagingSensor.get();
    }

}
