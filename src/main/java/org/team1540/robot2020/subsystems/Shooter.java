package org.team1540.robot2020.subsystems;

import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.TalonFXControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonFX;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Shooter extends SubsystemBase {
    private TalonFX shooterMotorA = new TalonFX(8);
    private TalonFX shooterMotorB = new TalonFX(9);

    private DigitalInput limitSwitch = new DigitalInput(0); // TODO:

    public Shooter() {
        setupMotors();
    }

    private void setupMotors() {
        // TODO figure out current limit
        shooterMotorA.configFactoryDefault();
        shooterMotorB.configFactoryDefault();

        shooterMotorA.setNeutralMode(NeutralMode.Coast);
        shooterMotorB.setNeutralMode(NeutralMode.Coast);

        shooterMotorB.follow(shooterMotorA);
    }

    @Override
    public void periodic() {
        SmartDashboard.putNumber("Shooter/shooterAVelocity", shooterMotorA.getSelectedSensorVelocity());
        SmartDashboard.putNumber("Shooter/shooterBVelocity", shooterMotorB.getSelectedSensorVelocity());
    }

    public void disableMotors() {
        shooterMotorA.set(TalonFXControlMode.PercentOutput, 0);
    }

    public double getVelocityTicksPerDecisecond() {
        return shooterMotorA.getSelectedSensorVelocity();
    }

    public void setVelocityTicksPerDecisecond(double velocity) {
        shooterMotorA.set(TalonFXControlMode.Velocity, velocity);
    }

    // Hood

    public boolean isLimitSwitchPressed() {
        return limitSwitch.get();
    }

    // Flywheel
    public double getFlywheelVelocity() {
        return shooterMotorA.getSelectedSensorVelocity();
    }

    public void setFlywheelVelocity(double velocity) {
        shooterMotorA.set(TalonFXControlMode.Velocity, velocity);
    }

    // Hood
    public double getHoodPercent() {
        return shooterMotorA.getMotorOutputPercent();
    }

    public void setHoodPercent(double speed) {
        shooterMotorA.set(TalonFXControlMode.PercentOutput, speed);
    }

    public double getHoodPosition() {
        return shooterMotorA.getSelectedSensorPosition();
    }

}
