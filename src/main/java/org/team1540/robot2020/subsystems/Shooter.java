package org.team1540.robot2020.subsystems;

import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.TalonFXControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonFX;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Shooter extends SubsystemBase {
    private TalonFX motorLeft = new TalonFX(0); // TODO
    private TalonFX motorRight = new TalonFX(1); // TODO
    private TalonFX motorHood = new TalonFX(2); // TODO

    private DigitalInput limitSwitch = new DigitalInput(0); // TODO

    public Shooter() {
        initMotors();
    }

    public void initMotors() {
        for (TalonFX controller : new TalonFX[]{motorLeft, motorRight}) {
            controller.setNeutralMode(NeutralMode.Brake);

            controller.enableVoltageCompensation(true);
            controller.configPeakOutputForward(1);
            controller.configPeakOutputReverse(-1);

            controller.configOpenloopRamp(0);
            controller.configForwardSoftLimitEnable(false);
            controller.configReverseSoftLimitEnable(false);
            controller.overrideLimitSwitchesEnable(false);
        }

        // TODO: Tune PID
        motorLeft.config_kP(0, 3);
        motorLeft.config_kI(0, 0.02);
        motorLeft.config_kD(0, 0);
        motorLeft.config_kF(0, 1);
    }

    // Flywheel
    public double getFlywheelVelocity() {
        return motorLeft.getSelectedSensorVelocity();
    }

    public void setFlywheelVelocity(double velocity) {
        motorLeft.set(TalonFXControlMode.Velocity, velocity);
    }

    // Hood
    public double getHoodPercent() {
        return motorHood.getMotorOutputPercent();
    }

    public void setHoodPercent(double speed) {
        motorHood.set(TalonFXControlMode.PercentOutput, speed);
    }

    public double getHoodPosition() {
        return motorHood.getSelectedSensorPosition();
    }

    public boolean limitSwitchPressed() {
        return limitSwitch.get();
    }
}
