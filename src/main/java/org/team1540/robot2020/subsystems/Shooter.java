package org.team1540.robot2020.subsystems;

import com.ctre.phoenix.motorcontrol.*;
import com.ctre.phoenix.motorcontrol.can.TalonFX;
import edu.wpi.first.networktables.EntryListenerFlags;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import org.team1540.robot2020.utils.MotorConfigUtils;

public class Shooter extends SubsystemBase {
    private double kP = 0.5;
    private double kD = 30;
    private double kF = 0.0484;
    private double kI = 0;
    private double iZone = 800.0;

    private TalonFX shooterMotorA = new TalonFX(9);
    private TalonFX shooterMotorB = new TalonFX(10);

    public Shooter() {
        setupFlywheelMotors();
        setupPIDs();
        updatePIDs();
    }

    private void setupFlywheelMotors() {
        MotorConfigUtils.setDefaultTalonFXConfig(shooterMotorA);
        MotorConfigUtils.setDefaultTalonFXConfig(shooterMotorB);
        shooterMotorA.configStatorCurrentLimit(new StatorCurrentLimitConfiguration(false, 0, 0, 0));
        shooterMotorB.configStatorCurrentLimit(new StatorCurrentLimitConfiguration(false, 0, 0, 0));
        shooterMotorA.setNeutralMode(NeutralMode.Coast);
        shooterMotorB.setNeutralMode(NeutralMode.Coast);
//        shooterMotorA.configVelocityMeasurementPeriod(VelocityMeasPeriod.Period_1Ms);
//        shooterMotorA.configVelocityMeasurementWindow(4);

        shooterMotorB.follow(shooterMotorA);
        shooterMotorB.setInverted(TalonFXInvertType.OpposeMaster);
    }

    private void setupPIDs() {
        SmartDashboard.putNumber("shooter/tuning/kP", kP);
        SmartDashboard.putNumber("shooter/kI", kI);
        SmartDashboard.putNumber("shooter/iZone", iZone);
        SmartDashboard.putNumber("shooter/tuning/kD", kD);
        SmartDashboard.putNumber("shooter/tuning/kF", kF);

        NetworkTableInstance.getDefault().getTable("SmartDashboard/shooter/tuning").addEntryListener((table, key, entry, value, flags) -> updatePIDs(), EntryListenerFlags.kUpdate);
    }

    private boolean wasPositive = true;

    @Override
    public void periodic() {
        SmartDashboard.putNumber("shooter/current", shooterMotorA.getStatorCurrent() + shooterMotorB.getStatorCurrent());
        SmartDashboard.putNumber("shooter/velocity", getVelocityRPM());
        SmartDashboard.putNumber("shooter/error", shooterMotorA.getClosedLoopError());
    }

    public void disableMotors() {
        shooterMotorA.set(TalonFXControlMode.PercentOutput, 0);
    }

    public double getVelocityRPM() {
        return (shooterMotorA.getSelectedSensorVelocity() / 2048.0) * 600;
    }

    public void setVelocityRPM(double velocity) {
        shooterMotorA.set(TalonFXControlMode.Velocity, (velocity * 2048.0) / 600);
    }

    public void updatePIDs() {
        shooterMotorA.config_kP(0, SmartDashboard.getNumber("shooter/tuning/kP", kP));
        shooterMotorA.config_kI(0, SmartDashboard.getNumber("shooter/kI", kI));
        shooterMotorA.config_IntegralZone(0, (int) SmartDashboard.getNumber("shooter/iZone", iZone));
        shooterMotorA.config_kD(0, SmartDashboard.getNumber("shooter/tuning/kD", kD));
        shooterMotorA.config_kF(0, SmartDashboard.getNumber("shooter/tuning/kF", kF));
    }

    public void setPercent(double value) {
        shooterMotorA.set(ControlMode.PercentOutput, value);
    }
}
