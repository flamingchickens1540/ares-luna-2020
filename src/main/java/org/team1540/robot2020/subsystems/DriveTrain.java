package org.team1540.robot2020.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import edu.wpi.first.wpilibj2.command.Subsystem;

public class DriveTrain implements Subsystem {

    CANSparkMax rightA;
    CANSparkMax rightB;
    CANSparkMax leftA;
    CANSparkMax leftB;

    public DriveTrain() {
        initDriveTrain();
    }

    public void set(double l, double r) {
        setL(l);
        setR(r);
    }

    public void setL(double vel) {
        leftA.set(vel);
    }
    public void setR(double vel) {
        rightA.set(vel);
    }

    public void initDriveTrain() {
        rightA = new CANSparkMax(1, MotorType.kBrushless);
        rightB = new CANSparkMax(2, MotorType.kBrushless);
        leftA = new CANSparkMax(3, MotorType.kBrushless);
        leftB = new CANSparkMax(4, MotorType.kBrushless);

        rightA.setInverted(true);
        leftA.setInverted(false);

        leftB.follow(leftA);
        rightB.follow(rightA);
    }
}
