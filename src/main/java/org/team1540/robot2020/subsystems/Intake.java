package org.team1540.robot2020.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Intake extends SubsystemBase {
    private CANSparkMax rollers = new CANSparkMax(0, CANSparkMaxLowLevel.MotorType.kBrushless);

    public void setRollerSpeed(double speed) {
        rollers.set(speed);
    }
}
