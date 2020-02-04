package org.team1540.robot2020.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonFX;
import edu.wpi.first.wpilibj.Servo;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Climber extends SubsystemBase {

    private static final double climberTicksPerMeter = 1;

    private TalonFX climberMotor = new TalonFX(12);
    private Servo ratchetServo = new Servo(0);

    public Climber() {
        // TODO figure out brake mode on all motors
        // TODO figure out current limit on all motors
        // TODO position PIDF tuning with networktables

        climberMotor.configFactoryDefault();
        setRatchet(RatchetState.ON);
    }

    @Override
    public void periodic() {
        SmartDashboard.putNumber("climberPosition", climberMotor.getSelectedSensorPosition());
        SmartDashboard.putNumber("climberVelocity", climberMotor.getSelectedSensorVelocity());
        SmartDashboard.putNumber("climberRatchetPosition", ratchetServo.get());
    }

    public void setPercent(double percent) {
        climberMotor.set(ControlMode.PercentOutput, percent);
    }

    public void disableMotors() {
        setPercent(0);
    }

    private double climberTicksToMeters(double ticks) {
        return ticks / climberTicksPerMeter;
    }

    private double climberMetersToTicks(double meters) {
        return meters * climberTicksPerMeter;
    }

    public void setPositionMeters(double meters) {
        climberMotor.set(ControlMode.Position, climberMetersToTicks(meters));
    }

    public double getPositionMeters() {
        return climberTicksToMeters(climberMotor.getSelectedSensorPosition());
    }

    public enum RatchetState {
        ON(1),
        OFF(0);

        private double servoPosition;

        RatchetState(double servoPosition) {
            this.servoPosition = servoPosition;
        }
    }

    public void setRatchet(RatchetState state) {
        ratchetServo.set(state.servoPosition);
    }
}
