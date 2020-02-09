package org.team1540.robot2020.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.StatorCurrentLimitConfiguration;
import com.ctre.phoenix.motorcontrol.can.SlotConfiguration;
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
    public double bottomOfBottomBallMeters = 0;
    public boolean isFull = false;

//  24569 ticks per 0.4318 meters
public static final double ticksPerMeter = 56899.0273275; // TODO NEVER change this
    public static final double ballSizeMeters = 0.1778;
    public static final double ballHeightThresholdMeters = 0.005;
    public static final double ballLengthsToIndexAfterShoot = 1.5;
    public static double firstIndexingSpeed = 0.5;
    public static double secondIndexingSpeed = 0.1;
    public static double afterMoveBallUpDist = 0;
    public static final double maxBalls = 5;

    public Indexer() {
        MotorConfigUtils.setDefaultTalonFXConfig(indexerMotor);
        SlotConfiguration defaultConfig = new SlotConfiguration();
        defaultConfig.kP = 1.0;
        defaultConfig.kI = 0.0;
        defaultConfig.kD = 0.0;
        defaultConfig.kF = 0.0;
        defaultConfig.integralZone = 0;
        defaultConfig.allowableClosedloopError = 0;
        defaultConfig.maxIntegralAccumulator = 0.0;
        indexerMotor.getSlotConfigs(defaultConfig, MotorConfigUtils.VELOCITY_SLOT_IDX, 50);
        indexerMotor.configGetStatorCurrentLimit(new StatorCurrentLimitConfiguration(true, 100, 0, 0));

        indexerMotor.setInverted(true);

        setEncoderTicks(0);

        SmartDashboard.putNumber("indexer/firstIndexingSpeed", firstIndexingSpeed);
        SmartDashboard.putNumber("indexer/secondIndexingSpeed", secondIndexingSpeed);
        SmartDashboard.putNumber("indexer/afterMoveBallUpDist", afterMoveBallUpDist);

        NetworkTableInstance.getDefault().getTable("SmartDashboard/indexer").addEntryListener((table, key, entry, value, flags) -> {
            Indexer.firstIndexingSpeed = SmartDashboard.getNumber("indexer/firstIndexingSpeed", Indexer.firstIndexingSpeed);
            Indexer.secondIndexingSpeed = SmartDashboard.getNumber("indexer/secondIndexingSpeed", Indexer.secondIndexingSpeed);
            Indexer.afterMoveBallUpDist = SmartDashboard.getNumber("indexer/afterMoveBallUpDist", Indexer.afterMoveBallUpDist);
        }, EntryListenerFlags.kUpdate);
    }

    @Override
    public void periodic() {
        SmartDashboard.putNumber("indexer/position/ticks", indexerMotor.getSelectedSensorPosition());
        SmartDashboard.putNumber("indexer/position/meters", this.getPositionMeters());
        SmartDashboard.putNumber("indexer/current", indexerMotor.getStatorCurrent());
        SmartDashboard.putNumber("indexer/position/closedLoopError", indexerMotor.getClosedLoopError());
        SmartDashboard.putNumber("indexer/position/closedLoopErrorMeters", this.getCloseLoopErrorMeters());

        SmartDashboard.putBoolean("indexer/indexerStagedSensor", getIndexerStagedSensor());
        SmartDashboard.putBoolean("indexer/shooterStagedSensor", getShooterStagedSensor());
        SmartDashboard.putNumber("indexer/balls", balls);
    }

    public void setPID(double p, double i, double d) {
        indexerMotor.config_kP(0, p);
        indexerMotor.config_kI(0, i);
        indexerMotor.config_kD(0, d);
    }

    public void setPercent(double percent) {
        // TODO: is it ok to config peak output every tick?
        indexerMotor.configPeakOutputForward(1);
        indexerMotor.configPeakOutputReverse(-1);
        indexerMotor.set(ControlMode.PercentOutput, percent);
    }

    public void setPositionMeters(double positionMeters, double speed) {
        // TODO: is it ok to config peak output every tick?
        indexerMotor.configPeakOutputForward(speed);
        indexerMotor.configPeakOutputReverse(-speed);
        indexerMotor.set(ControlMode.Position, positionMeters * ticksPerMeter);
    }

    public boolean getIndexerStagedSensor() {
        return !indexerStagedSensor.get();
    }

    public boolean getShooterStagedSensor() {
        return !shooterStagedSensor.get();
    }

    public double getPositionMeters() {
        // TODO this needs tuning constant to convert from ticks
        return indexerMotor.getSelectedSensorPosition() / ticksPerMeter;
    }

    public double getCloseLoopErrorMeters() {
        return indexerMotor.getClosedLoopError() / ticksPerMeter;
    }

    public void setEncoderTicks(int position) {
        indexerMotor.setSelectedSensorPosition(position);
    }

    public int getBalls() {
        return balls;
    }

//    public boolean isFull() {
//        return getBalls() == maxBalls;
//    }

    public void ballAdded() {
        balls++;
    }

    public void ballRemoved() {
        balls--;
    }
}
