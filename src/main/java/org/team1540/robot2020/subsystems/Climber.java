package org.team1540.robot2020.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonFX;
import edu.wpi.first.wpilibj.Servo;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Climber extends SubsystemBase {
    private TalonFX climberMotor = new TalonFX(12);

    private Servo ratchetServo = new Servo(0);

    public static final double positionThresholdMeters = 0.05;

    public Climber() {
        // TODO figure out brake mode on all motors
        // TODO figure out current limit on all motors
        climberMotor.configFactoryDefault();

        setRatchetServo(true);
    }

    public void setPercent(double percent) {
        climberMotor.set(ControlMode.PercentOutput, percent);
    }

    public void disableMotors() {
        setPercent(0);
    }

    public double ticksToMeters(double ticks) {
        return ticks;
    }

    // TODO these methods need to set and get climber position in meters, not ticks
    // todo ticks to climber pos meters method
    public void setPositionTicks(double ticks) {
        climberMotor.set(ControlMode.Position, ticks);
    }

    public void setPositionMeters(double meters) {
        setPositionTicks(ticksToMeters(meters));
    }

    public double getPositionTicks() {
        return climberMotor.getSelectedSensorPosition();
    }

    public double getPositionMeters() {
        return ticksToMeters(getPositionTicks());
    }

    public boolean isAtPositionMeters(double meters) {
        return Math.abs(getPositionMeters() - meters) <= positionThresholdMeters;
    }

    public void setRatchetServo(boolean on) {
        ratchetServo.set(on ? 1 : 0);
    }

}
