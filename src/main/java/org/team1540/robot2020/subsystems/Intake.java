package org.team1540.robot2020.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Intake extends SubsystemBase {
    // TODO figure out brake mode on all motors
    // TODO figure out current limit on all motors
    private CANSparkMax rollers = new CANSparkMax(4, CANSparkMaxLowLevel.MotorType.kBrushless);

    private CANSparkMax funnelA = new CANSparkMax(5, CANSparkMaxLowLevel.MotorType.kBrushless);
    private CANSparkMax funnelB = new CANSparkMax(6, CANSparkMaxLowLevel.MotorType.kBrushless);

    public Intake() {
        rollers.restoreFactoryDefaults();

        funnelA.restoreFactoryDefaults();
        funnelB.restoreFactoryDefaults();
    }

    public void setRollerSpeed(double speed) {
        rollers.set(speed);
    }

    public void setFunnelSpeed(double speed) {
        setFunnelSpeed(speed, speed);
    }

    public void setFunnelSpeed(double speedA, double speedB) {
        funnelA.set(speedA);
        funnelB.set(speedB);
    }

    public void setFunnelAndRollerSpeed(double speed) {
        setRollerSpeed(speed);
        setFunnelSpeed(speed);
    }

    // TODO add stop
}
