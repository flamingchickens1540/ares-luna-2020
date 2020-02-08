package org.team1540.robot2020.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonFX;
import com.ctre.phoenix.motorcontrol.can.TalonFXConfiguration;
import edu.wpi.first.wpilibj.Servo;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import org.team1540.robot2020.utils.MotorConfigUtils;

public class Climber extends SubsystemBase {

    public static final double climberTicksPerMeter = 1;
    public static final double climberTopPositionMeters = 1;

    // TODO CHANGE CURRENT DRAW, TIME AND VELOCITY THRESHOLD ONCE TESTING IS COMPLETE
    public static final double currentThreshold = 1;
    public static final double velocityThreshold = 1;
    public static final double timeThreshold = 0.15;

    private TalonFX climberMotor = new TalonFX(13);
    private Servo ratchetServo = new Servo(9);

    private TalonFXConfiguration defaultConfig = MotorConfigUtils.get1540DefaultTalonFXConfiguration();

    public Climber() {
        // TODO figure out brake mode on all motors
        // TODO figure out current limit on all motors
        // TODO position PIDF tuning with networktables


        // TODO tune PIDF values
        defaultConfig.slot1.kP = 1;
        defaultConfig.slot1.kI = 0;
        defaultConfig.slot1.kD = 0;
        defaultConfig.slot1.kF = 0;

        climberMotor.configAllSettings(defaultConfig);

        setRatchet(RatchetState.ON);
    }

    @Override
    public void periodic() {
        SmartDashboard.putNumber("climber/position", climberMotor.getSelectedSensorPosition());
        SmartDashboard.putNumber("climber/velocity", climberMotor.getSelectedSensorVelocity());
        SmartDashboard.putNumber("climber/ratchetPosition", ratchetServo.get());
    }

    public void setPercent(double percent) {
        climberMotor.set(ControlMode.PercentOutput, percent);
    }

    public void stop() {
        climberMotor.set(ControlMode.PercentOutput,0);
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
        // TODO tune gravity feed-forward
        // TODO we will probably need a different control method for the actual climb, because we should disengage the
        //  motors and let the ratchet hold us once we're high enough and also because the PIDs for a ~5 pound climber
        //  frame are probably different than those for a 140-pound robot (and a PID might not actually be necessary)
        climberMotor.set(ControlMode.MotionMagic, climberMetersToTicks(meters));
    }

    public double getCurrentDraw() {
        return climberMotor.getStatorCurrent();
    }

    public double getVelocity() {
        return climberMotor.getSelectedSensorVelocity();
    }

    public double getPositionMeters() {
        return climberTicksToMeters(climberMotor.getSelectedSensorPosition());
    }

    public boolean atPositionMeters(double position, double toleranceMeters) {
        return Math.abs(getPositionMeters() - position) <= toleranceMeters;
    }

    public enum RatchetState {
        ON(0.372),
        OFF(0);

        private double servoPosition;

        RatchetState(double servoPosition) {
            this.servoPosition = servoPosition;
        }
    }

    // TODO maybe set the max output in the "up" direction to 0 whenever the ratchet is engaged
    public void setRatchet(boolean state) {
        if(state) ratchetServo.set(RatchetState.ON.servoPosition); else ratchetServo.set(RatchetState.OFF.servoPosition);
    }

    public void setRatchet(double position) {
        ratchetServo.set(position);
    }
}
