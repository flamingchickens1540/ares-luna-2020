package org.team1540.robot2020.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Indexer extends SubsystemBase {
    private CANSparkMax cord = new CANSparkMax(0, CANSparkMaxLowLevel.MotorType.kBrushless);
    private int balls = 0;

    public void setSpeed(double speed) {
        cord.set(speed);
    }

    public int getBalls() {
        return balls;
    }

    public void addBall() {
        balls++;
    }

    public void removeBall() {
        balls--;
    }
}
