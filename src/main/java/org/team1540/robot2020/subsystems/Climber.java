package org.team1540.robot2020.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonFX;
import edu.wpi.first.wpilibj.Servo;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Climber extends SubsystemBase {
    private TalonFX climberMotor = new TalonFX(12);

    private Servo ratchetServo = new Servo(0);

    public Climber() {
        // TODO figure out brake mode on all motors
        // TODO figure out current limit on all motors
        climberMotor.configFactoryDefault();

        setRatchetServo(true);
    }

    public void setPercent(double percent) {
        climberMotor.set(ControlMode.PercentOutput, percent);
    }

    // TODO these methods need to set and get climber position in meters, not ticks
    // todo ticks to climber pos meters method
    public void setPositionTicks(double position) {
        climberMotor.set(ControlMode.Position, position);
    }

    public void setPositionMeters(double position) {
        climberMotor.set(ControlMode.Position, position);
    }

    public double getPositionTicks() {
        return climberMotor.getSelectedSensorPosition();
    }

    public double getPositionMeters() {
        return climberMotor.getSelectedSensorPosition();
    }

    // todo this method should not deal with ticks, use meters instead
    // todo this should take the tolerance in meters
    public boolean isAtPositionTicks(double position) {
        return Math.abs(getPositionTicks() - position) <= 50;
    }

    public boolean isAtPositionMeters(double position) {
        return Math.abs(getPositionTicks() - position) <= 50;
    }

    public void setRatchetServo(boolean on) {
        ratchetServo.set(on ? 1 : 0);
    }

    // todo disableMotors method to set percent to zero

}
