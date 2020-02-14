package org.team1540.robot2020.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.StatorCurrentLimitConfiguration;
import com.ctre.phoenix.motorcontrol.can.TalonFX;
import edu.wpi.first.networktables.EntryListenerFlags;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.Servo;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import org.team1540.robot2020.utils.MotorConfigUtils;

import static org.team1540.robot2020.utils.MotorConfigUtils.POSITION_SLOT_IDX;

public class Climber extends SubsystemBase {

    public static final double climberTicksPerMeter = 175289.47806139;
    public static final double climberTopPositionMeters = 0.7;

    private double kP = 0.01;
    private double kI = 0;
    private double kD;
    private double kF = 0.04938;

    private TalonFX climberMotor = new TalonFX(13);
    private Servo ratchetServo = new Servo(9);


    public Climber() {
        // TODO figure out brake mode on all motors
        // TODO figure out current limit on all motors
        // TODO position PIDF tuning with networktables


        // TODO tune PIDF values

        MotorConfigUtils.setDefaultTalonFXConfig(climberMotor);

        climberMotor.selectProfileSlot(POSITION_SLOT_IDX, 0);

        climberMotor.configGetStatorCurrentLimit(new StatorCurrentLimitConfiguration(false, 0, 0, 0));


        climberMotor.setInverted(true);

        climberMotor.setNeutralMode(NeutralMode.Brake);

        setRatchet(RatchetState.DISENGAGED);
        SmartDashboard.putNumber("climber/tuning/kP", kP);
        SmartDashboard.putNumber("climber/tuning/kI", kI);
        SmartDashboard.putNumber("climber/tuning/kD", kD);
        SmartDashboard.putNumber("climber/tuning/kF", kF);
        SmartDashboard.putNumber("climber/tuning/closedLoopRamp", 0.1);
        SmartDashboard.putNumber("climber/tuning/configMotionAcceleration", 100000);
        SmartDashboard.putNumber("climber/tuning/configMotionCruiseVelocity", 20000);

        updatePIDs();
        NetworkTableInstance.getDefault().getTable("SmartDashboard/climber/tuning").addEntryListener((table, key, entry, value, flags) -> updatePIDs(), EntryListenerFlags.kUpdate);
    }

    private void updatePIDs() {
        climberMotor.configClosedloopRamp(SmartDashboard.getNumber("climber/tuning/closedLoopRamp", 0));
        climberMotor.config_kP(POSITION_SLOT_IDX, SmartDashboard.getNumber("climber/tuning/kP", kP));
        climberMotor.config_kI(POSITION_SLOT_IDX, SmartDashboard.getNumber("climber/tuning/kI", kI));
        climberMotor.config_kD(POSITION_SLOT_IDX, SmartDashboard.getNumber("climber/tuning/kD", kD));
        climberMotor.config_kF(POSITION_SLOT_IDX, SmartDashboard.getNumber("climber/tuning/kF", kF));
        climberMotor.configMotionAcceleration((int) SmartDashboard.getNumber("climber/tuning/configMotionAcceleration", 0));
        climberMotor.configMotionCruiseVelocity((int) SmartDashboard.getNumber("climber/tuning/configMotionCruiseVelocity", 0));
    }

    public void zero() {
        climberMotor.setSelectedSensorPosition(0);
    }

    @Override
    public void periodic() {
        SmartDashboard.putNumber("climber/position", getPositionMeters());
        SmartDashboard.putNumber("climber/velocity", climberMotor.getSelectedSensorVelocity());
        SmartDashboard.putNumber("climber/error", climberMotor.getClosedLoopError());
        SmartDashboard.putNumber("climber/current", climberMotor.getStatorCurrent());
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
        climberMotor.set(ControlMode.MotionMagic, climberMetersToTicks(meters));
    }

    public double getCurrentDraw() {
        return climberMotor.getStatorCurrent();
    }

    public double getVelocityMeters() {
        return climberTicksToMeters(climberMotor.getSelectedSensorVelocity() * 10);
    }

    public double getPositionMeters() {
        return climberTicksToMeters(climberMotor.getSelectedSensorPosition());
    }

    public boolean atPositionMeters(double position, double toleranceMeters) {
        return Math.abs(getPositionMeters() - position) <= toleranceMeters;
    }

    public enum RatchetState {
        ENGAGED(0),
        DISENGAGED(0.372);

        private double servoPosition;

        RatchetState(double servoPosition) {
            this.servoPosition = servoPosition;
        }
    }

    public void setRatchet(RatchetState state) {
        climberMotor.configPeakOutputForward(state == RatchetState.ENGAGED ? 0 : 1);
        ratchetServo.set(state.servoPosition);
    }

    public void setRatchet(double position) {
        ratchetServo.set(position);
    }

    public void setBrake(NeutralMode state) {
        climberMotor.setNeutralMode(state);
    }
}
