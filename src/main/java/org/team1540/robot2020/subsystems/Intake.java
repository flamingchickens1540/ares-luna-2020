package org.team1540.robot2020.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Intake extends SubsystemBase {
    private CANSparkMax outsideRoller = new CANSparkMax(0, CANSparkMaxLowLevel.MotorType.kBrushless);
    private CANSparkMax insideRollers = new CANSparkMax(0, CANSparkMaxLowLevel.MotorType.kBrushless);

    public void setInsideRollers(double speed) {
        insideRollers.set(speed);
    }

    public void setOutsideRoller(double speed) {
        outsideRoller.set(speed);
    }
}
