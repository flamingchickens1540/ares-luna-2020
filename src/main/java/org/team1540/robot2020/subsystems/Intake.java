package org.team1540.robot2020.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Intake extends SubsystemBase {
    private CANSparkMax rollers = new CANSparkMax(4, CANSparkMaxLowLevel.MotorType.kBrushless);

    private CANSparkMax funnelA = new CANSparkMax(5, CANSparkMaxLowLevel.MotorType.kBrushless);
    private CANSparkMax funnelB = new CANSparkMax(6, CANSparkMaxLowLevel.MotorType.kBrushless);

    public void setRollerSpeed(double speed) {
        rollers.set(speed);
    }

    public void setFunnelASpeed(double speed) {
        funnelA.set(speed);
    }

    public void setFunnelBSpeed(double speed) {
        funnelB.set(speed);
    }

    public void setFunnelSpeed(double speed) {
        setFunnelASpeed(speed);
        setFunnelBSpeed(speed);
    }

    public void setSpeed(double speed) {
        setRollerSpeed(speed);
        setFunnelSpeed(speed);
    }
}
