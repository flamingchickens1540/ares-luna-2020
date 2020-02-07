package org.team1540.robot2020.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonFX;
import com.ctre.phoenix.motorcontrol.can.TalonFXConfiguration;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import org.team1540.robot2020.utils.MotorConfigUtils;

public class Indexer extends SubsystemBase {
    private TalonFX indexerMotor = new TalonFX(8);

    private DigitalInput indexerStagedSensor = new DigitalInput(0);
    private DigitalInput shooterStagedSensor = new DigitalInput(1);

    private int balls = 0;

    public static final double ballLengthsToIndexAfterShoot = 1.5;
    public static final double ballSizeMeters = 0.2;
    public static final double indexingSpeed = 1;
    public static final double ballHeightThresholdMeters = 0.05;
    public static final double maxBalls = 5;

    public Indexer() {
        TalonFXConfiguration defaultConfig = MotorConfigUtils.get1540DefaultTalonFXConfiguration();
        defaultConfig.slot1.kP = 0;
        defaultConfig.slot1.kI = 0;
        defaultConfig.slot1.kD = 0;
        indexerMotor.configAllSettings(defaultConfig);
    }

    @Override
    public void periodic() {
        SmartDashboard.putBoolean("indexer/indexerStaged", indexerStagedSensor.get());
        SmartDashboard.putBoolean("indexer/shooterStaged", shooterStagedSensor.get());
        SmartDashboard.putNumber("indexer/balls", balls);
    }

    public void setPercent(double percent) {
        indexerMotor.set(ControlMode.PercentOutput, percent);
    }

    public boolean getIndexerStagedSensor() {
        return indexerStagedSensor.get();
    }

    public boolean getShooterStagedSensor() {
        return shooterStagedSensor.get();
    }

    public double getEncoderMeters() {
        // TODO this needs tuning constant to convert from ticks
        return indexerMotor.getSelectedSensorPosition();
    }

    public int getBalls() {
        return balls;
    }

    public boolean isFull() {
        return getBalls() == maxBalls;
    }

    public void ballAdded() {
        balls++;
    }

    public void ballRemoved() {
        balls--;
    }
}
