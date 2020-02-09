package org.team1540.robot2020.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.StatorCurrentLimitConfiguration;
import com.ctre.phoenix.motorcontrol.can.SlotConfiguration;
import com.ctre.phoenix.motorcontrol.can.TalonFX;
import edu.wpi.first.wpilibj.Servo;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import org.team1540.robot2020.utils.MotorConfigUtils;

import static org.team1540.robot2020.utils.MotorConfigUtils.VELOCITY_SLOT_IDX;

public class Climber extends SubsystemBase {

    public static final double climberTicksPerMeter = 175289.47806139;
    public static final double climberTopPositionMeters = 0.7;

    // TODO CHANGE CURRENT DRAW, TIME AND VELOCITY THRESHOLD ONCE TESTING IS COMPLETE
    public static final double currentThreshold = 1;
    public static final double velocityThreshold = 1;
    public static final double timeThreshold = 0.15;

    private TalonFX climberMotor = new TalonFX(13);
    private Servo ratchetServo = new Servo(9);
    private double offset = 0;


    public Climber() {
        // TODO figure out brake mode on all motors
        // TODO figure out current limit on all motors
        // TODO position PIDF tuning with networktables


        // TODO tune PIDF values

        MotorConfigUtils.setDefaultTalonFXConfig(climberMotor);

        SlotConfiguration defaultConfig = new SlotConfiguration();
        defaultConfig.kP = 1;
        defaultConfig.kI = 0;
        defaultConfig.kD = 0;
        defaultConfig.kF = 0;
        climberMotor.getSlotConfigs(defaultConfig, VELOCITY_SLOT_IDX, 50);
        climberMotor.selectProfileSlot(VELOCITY_SLOT_IDX, 0);

        climberMotor.configGetStatorCurrentLimit(new StatorCurrentLimitConfiguration(false, 0, 0, 0));

        climberMotor.setInverted(true);

        setRatchet(true);
    }

    public void zero() {
        offset = -climberMotor.getSelectedSensorPosition();
    }

    @Override
    public void periodic() {
        SmartDashboard.putNumber("climber/position", getPositionMeters());
        SmartDashboard.putNumber("climber/velocity", climberMotor.getSelectedSensorVelocity());
        SmartDashboard.putNumber("climber/ratchetPosition", ratchetServo.get());
        SmartDashboard.putNumber("climber/throttle", climberMotor.getMotorOutputPercent());
    }

    public void setPercent(double percent) {
        climberMotor.set(ControlMode.PercentOutput, percent);
    }

    public void stop() {
        climberMotor.set(ControlMode.PercentOutput, 0);
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
        return climberTicksToMeters(climberMotor.getSelectedSensorPosition() + offset);
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
        ratchetServo.set(state ? RatchetState.ON.servoPosition : RatchetState.OFF.servoPosition);
    }

    public void setRatchet(double position) {
        ratchetServo.set(position);
    }

    public void setBrake(NeutralMode state) {
        climberMotor.setNeutralMode(state);
    }

    public void setBrake(boolean state) {
        climberMotor.setNeutralMode(state ? NeutralMode.Brake : NeutralMode.Coast);
    }
}
