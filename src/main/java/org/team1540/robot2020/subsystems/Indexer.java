package org.team1540.robot2020.subsystems;

import com.revrobotics.CANEncoder;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Indexer extends SubsystemBase {
    private CANSparkMax indexerMotor = new CANSparkMax(7, CANSparkMaxLowLevel.MotorType.kBrushless);
    private CANEncoder indexerEncoder = indexerMotor.getEncoder();

    private DigitalInput indexerStagedSensor = new DigitalInput(0);
    private DigitalInput shooterStagedSensor = new DigitalInput(1);

    private int balls = 0;

    public static final double ballLengthsToIndexAfterShoot = 1.5;

    public Indexer() {
        // TODO figure out current limit on all motors
        indexerMotor.restoreFactoryDefaults();

        indexerMotor.setIdleMode(CANSparkMax.IdleMode.kBrake);
    }

    @Override
    public void periodic() {
        SmartDashboard.putBoolean("indexer/indexerStaged", indexerStagedSensor.get());
        SmartDashboard.putBoolean("indexer/shooterStaged", shooterStagedSensor.get());
        SmartDashboard.putNumber("indexer/balls", balls);
    }

    public void setPercent(double percent) {
        indexerMotor.set(percent);
    }

    public boolean getIndexerStagedSensor() {
        return indexerStagedSensor.get();
    }

    public boolean getShooterStagedSensor() {
        return shooterStagedSensor.get();
    }

    // TODO this should use meters instead of inches
    public double getEncoderInches() {
        // TODO this needs tuning constant to convert from ticks
        return indexerEncoder.getPosition();
    }

    private void resetEncoder() {
        indexerEncoder.setPosition(0);
    }

    public int getBalls() {
        return balls;
    }

    public boolean isFull() {
        // TODO this should be tunable
        return getBalls() == 5;
    }

    public void ballAdded() {
        balls++;
    }

    public void ballRemoved() {
        balls--;
    }
}
