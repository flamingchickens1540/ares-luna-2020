package org.team1540.robot2020.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.TalonFXControlMode;
import com.ctre.phoenix.motorcontrol.TalonFXInvertType;
import com.ctre.phoenix.motorcontrol.can.TalonFX;
import edu.wpi.first.networktables.EntryListenerFlags;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import org.team1540.robot2020.utils.MotorConfigUtils;

public class Shooter extends SubsystemBase {
    private final double kP = 2.5;
    private final double kD = 0;
    private final double kF = 0.0518;

    private TalonFX shooterMotorA = new TalonFX(9);
    private TalonFX shooterMotorB = new TalonFX(10);

    public Shooter() {
        setupFlywheelMotors();
        setupPIDs();
        updatePIDs();
    }

    private void setupFlywheelMotors() {
        shooterMotorA.setNeutralMode(NeutralMode.Coast);
        shooterMotorB.setNeutralMode(NeutralMode.Coast);

        shooterMotorB.follow(shooterMotorA);
        shooterMotorB.setInverted(TalonFXInvertType.OpposeMaster);
    }

    private void setupPIDs() {
        SmartDashboard.putNumber("shooter/kP", kP);
        SmartDashboard.putNumber("shooter/kD", kD);
        SmartDashboard.putNumber("shooter/kF", kF);

        NetworkTableInstance.getDefault().getTable("SmartDashboard/shooter").addEntryListener((table, key, entry, value, flags) -> updatePIDs(), EntryListenerFlags.kUpdate);
    }

    @Override
    public void periodic() {
        SmartDashboard.putNumber("shooter/current", shooterMotorA.getStatorCurrent() + shooterMotorB.getStatorCurrent());
        SmartDashboard.putNumber("shooter/velocity", getVelocity());
        SmartDashboard.putNumber("shooter/error", shooterMotorA.getClosedLoopError());
    }

    public void disableMotors() {
        shooterMotorA.set(TalonFXControlMode.PercentOutput, 0);
    }

    public double getVelocity() {
        return (shooterMotorA.getSelectedSensorVelocity() / 2048.0) * 600;
    }

    public void setVelocity(double velocity) {
        shooterMotorA.set(TalonFXControlMode.Velocity, (velocity * 2048.0) / 600);
    }

    public void updatePIDs() {
        shooterMotorA.config_kP(0, SmartDashboard.getNumber("shooter/kP", kP));
        shooterMotorA.config_kD(0, SmartDashboard.getNumber("shooter/kD", kD));
        shooterMotorA.config_kF(0, SmartDashboard.getNumber("shooter/kF", kF));
    }

    public void setPercent(double value) {
        shooterMotorA.set(ControlMode.PercentOutput, value);
    }
}
