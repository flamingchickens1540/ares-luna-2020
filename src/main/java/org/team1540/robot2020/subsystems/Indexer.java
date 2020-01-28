package org.team1540.robot2020.subsystems;

import com.revrobotics.CANEncoder;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel;
import edu.wpi.first.wpilibj.AnalogInput;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.controller.PIDController;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Indexer extends SubsystemBase {
    private CANSparkMax indexerMotor = new CANSparkMax(0, CANSparkMaxLowLevel.MotorType.kBrushless);
    private CANEncoder indexerMotorEncoder = indexerMotor.getEncoder();

    private DigitalInput indexerStaged = new DigitalInput(0);
    private DigitalInput shooterStaged = new DigitalInput(1);

    private int balls = 0;

    public Indexer() {
        indexerMotor.setIdleMode(CANSparkMax.IdleMode.kBrake);
    }

    public void setPercent(double speed) {
        indexerMotor.set(speed);
    }

    public boolean getIndexerStaged() {
        return indexerStaged.get();
    }

    public boolean getShooterStaged() {
        return shooterStaged.get();
    }

    public double getEncoderInches() {
        return indexerMotorEncoder.getPosition();
    }

    public void resetEncoder() {
        indexerMotorEncoder.setPosition(0);
    }

    public int getBalls() {
        return balls;
    }

    public boolean isFull() {
        return getBalls() == 5;
    }

    public void ballAdded() {
        balls++;
    }

    public void ballRemoved() {
        balls--;
    }
}
