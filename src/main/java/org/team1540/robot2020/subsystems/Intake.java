package org.team1540.robot2020.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Intake extends SubsystemBase {
    private CANSparkMax rollers = new CANSparkMax(4, CANSparkMaxLowLevel.MotorType.kBrushless);

    private CANSparkMax funnelA = new CANSparkMax(5, CANSparkMaxLowLevel.MotorType.kBrushless);
    private CANSparkMax funnelB = new CANSparkMax(6, CANSparkMaxLowLevel.MotorType.kBrushless);

    public Intake() {
        funnelB.follow(funnelA);
    }

    public void setRollerFunnelSpeed(double speed) {
        rollers.set(speed);
        funnelA.set(speed);
    }
}
