package org.team1540.robot2020.subsystems;

import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.TalonFXControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonFX;
import com.ctre.phoenix.motorcontrol.can.TalonFXConfiguration;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import org.team1540.robot2020.utils.MotorConfigUtils;

public class Shooter extends SubsystemBase {
    private TalonFX shooterMotorA = new TalonFX(8);
    private TalonFX shooterMotorB = new TalonFX(9);

    // TODO the shooter hood limit switch is plugged into the reverse limit switch port of the spark max controlling the hood, see the Ares/Luna controls google sheet pinned in #robot-software
    private DigitalInput limitSwitch = new DigitalInput(0);

    public Shooter() {
        setupFlywheelMotors();
        setupHoodMotors();
    }

    private void setupHoodMotors() {
        // TODO
    }

    private void setupFlywheelMotors() {
        // TODO figure out current limit
        TalonFXConfiguration defaultConfig = MotorConfigUtils.get1540DefaultTalonFXConfiguration();
        shooterMotorA.configAllSettings(defaultConfig);
        shooterMotorB.configAllSettings(defaultConfig);

        shooterMotorA.setNeutralMode(NeutralMode.Coast);
        shooterMotorB.setNeutralMode(NeutralMode.Coast);

        shooterMotorB.follow(shooterMotorA);
    }

    @Override
    public void periodic() {
        SmartDashboard.putNumber("shooter/shooterVelocityA", shooterMotorA.getSelectedSensorVelocity());
        SmartDashboard.putNumber("shooter/shooterVelocityA", shooterMotorB.getSelectedSensorVelocity());
        // TODO log shooter current draw, hood position, hood current draw
    }

    public void disableMotors() {
        shooterMotorA.set(TalonFXControlMode.PercentOutput, 0);
    }

    // Flywheel
    public double getFlywheelVelocityTicksPerDecisecond() {
        return shooterMotorA.getSelectedSensorVelocity();
    }

    public void setFlywheelVelocityTicksPerDecisecond(double velocity) {
        shooterMotorA.set(TalonFXControlMode.Velocity, velocity);
    }

    // Hood
    public double getHoodPercent() {
        // TODO: this is setting the shooter motor, not the hood motor
        return shooterMotorA.getMotorOutputPercent();
    }

    public void setHoodPercent(double speed) {
        // TODO: this is setting the shooter motor, not the hood motor
        shooterMotorA.set(TalonFXControlMode.PercentOutput, speed);
    }

    public double getHoodPosition() {
        // TODO: this is setting the shooter motor, not the hood motor
        return shooterMotorA.getSelectedSensorPosition();
    }

    // Hood Limit Switch
    public boolean isLimitSwitchPressed() {
        return limitSwitch.get();
    }

}
