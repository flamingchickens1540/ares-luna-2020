package org.team1540.robot2020.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.InvertType;
import com.ctre.phoenix.motorcontrol.can.TalonFX;
import edu.wpi.first.networktables.EntryListenerFlags;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import org.team1540.robot2020.utils.MotorConfigUtils;

public class Indexer extends SubsystemBase {
    private TalonFX indexerMotor = new TalonFX(8);

    private DigitalInput indexerStagedSensor = new DigitalInput(0);
    private DigitalInput shooterStagedSensor = new DigitalInput(1);

    private int balls = 0;
//  24569 ticks per 0.4318 meters
    public static final double ticksPerMeter = 56899.0273275;
    public static final double ballSizeMeters = 0.1778;
    public static final double ballHeightThresholdMeters = 0.005;
    public static final double ballLengthsToIndexAfterShoot = 1.5;
    public static double firstIndexingSpeed = 0.5;
    public static double secondIndexingSpeed = 0.1;
    public static final double maxBalls = 5;
    public double bottomOfBottomBallMeters;

    public Indexer() {
        MotorConfigUtils.setDefaultTalonFXConfig(indexerMotor);
        indexerMotor.setInverted(InvertType.InvertMotorOutput);

        SmartDashboard.putNumber("indexer/firstIndexingSpeed", Indexer.firstIndexingSpeed);
        SmartDashboard.putNumber("indexer/secondIndexingSpeed", Indexer.secondIndexingSpeed);

        NetworkTableInstance.getDefault().getTable("SmartDashboard/indexer").addEntryListener((table, key, entry, value, flags) -> {
            Indexer.firstIndexingSpeed = SmartDashboard.getNumber("indexer/firstIndexingSpeed", Indexer.firstIndexingSpeed);
            Indexer.secondIndexingSpeed = SmartDashboard.getNumber("indexer/secondIndexingSpeed", Indexer.secondIndexingSpeed);
        }, EntryListenerFlags.kUpdate);
    }


    @Override
    public void periodic() {
        SmartDashboard.putNumber("indexer/encoder", indexerMotor.getSelectedSensorPosition());
        SmartDashboard.putNumber("indexer/current", indexerMotor.getStatorCurrent());

        SmartDashboard.putBoolean("indexer/indexerStaged", getIndexerStagedSensor());
        SmartDashboard.putBoolean("indexer/shooterStaged", getShooterStagedSensor());
        SmartDashboard.putNumber("indexer/balls", balls);
    }

    public void setPercent(double percent) {
        indexerMotor.set(ControlMode.PercentOutput, percent);
    }

    public boolean getIndexerStagedSensor() {
        return !indexerStagedSensor.get();
    }

    public boolean getShooterStagedSensor() {
        return !shooterStagedSensor.get();
    }

    public double getEncoderMeters() {
        // TODO this needs tuning constant to convert from ticks
        return indexerMotor.getSelectedSensorPosition() / ticksPerMeter;
    }

    public void setEncoderTicks(int position) {
        indexerMotor.setSelectedSensorPosition(position);
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
