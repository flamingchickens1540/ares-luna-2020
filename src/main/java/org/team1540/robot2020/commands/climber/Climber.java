package org.team1540.robot2020.commands.climber;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.StatorCurrentLimitConfiguration;
import com.ctre.phoenix.motorcontrol.can.TalonFX;
import edu.wpi.first.networktables.EntryListenerFlags;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.AnalogInput;
import edu.wpi.first.wpilibj.Servo;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import org.team1540.robot2020.utils.MotorConfigUtils;

import static org.team1540.robot2020.utils.MotorConfigUtils.POSITION_SLOT_IDX;

public class Climber extends SubsystemBase {

    public static final double HOOK_MIN_LOCATION = 0.6;

    private static final double climberTicksPerMeter = 175289.47806139;
    private static final double closedLoopRamp = 0.1;
    private static final int maxAcceleration = 15000;
    private static final int maxVelocity = 10000;

    private static final double kP = 0.01;
    private static final double kI = 0;
    private static final double kD = 0;
    private static final double kF = 0.04938;

    private TalonFX climberMotor = new TalonFX(13);
    private Servo ratchetServo = new Servo(9);
    private AnalogInput distanceSensor = new AnalogInput(0);


    public Climber() {
        MotorConfigUtils.setDefaultTalonFXConfig(climberMotor);
        climberMotor.selectProfileSlot(POSITION_SLOT_IDX, 0);
        climberMotor.configGetStatorCurrentLimit(new StatorCurrentLimitConfiguration(false, 0, 0, 0));
        climberMotor.setInverted(true);

        setRatchet(RatchetState.DISENGAGED);

        SmartDashboard.putNumber("climber/tuning/kP", kP);
        SmartDashboard.putNumber("climber/tuning/kI", kI);
        SmartDashboard.putNumber("climber/tuning/kD", kD);
        SmartDashboard.putNumber("climber/tuning/kF", kF);
        SmartDashboard.putNumber("climber/tuning/closedLoopRamp", closedLoopRamp);
        SmartDashboard.putNumber("climber/tuning/configMotionAcceleration", maxAcceleration);
        SmartDashboard.putNumber("climber/tuning/configMotionCruiseVelocity", maxVelocity);

        configSoftLimitMeters(0.05, 0.75); // TODO: tune these soft limits
        updatePIDs();
        NetworkTableInstance.getDefault().getTable("SmartDashboard/climber/tuning").addEntryListener((table, key, entry, value, flags) -> updatePIDs(), EntryListenerFlags.kUpdate);
    }

    private void updatePIDs() {
        climberMotor.config_kP(POSITION_SLOT_IDX, SmartDashboard.getNumber("climber/tuning/kP", kP));
        climberMotor.config_kI(POSITION_SLOT_IDX, SmartDashboard.getNumber("climber/tuning/kI", kI));
        climberMotor.config_kD(POSITION_SLOT_IDX, SmartDashboard.getNumber("climber/tuning/kD", kD));
        climberMotor.config_kF(POSITION_SLOT_IDX, SmartDashboard.getNumber("climber/tuning/kF", kF));
        climberMotor.configClosedloopRamp(SmartDashboard.getNumber("climber/tuning/closedLoopRamp", closedLoopRamp));
        climberMotor.configMotionAcceleration((int) SmartDashboard.getNumber("climber/tuning/configMotionAcceleration", maxAcceleration));
        climberMotor.configMotionCruiseVelocity((int) SmartDashboard.getNumber("climber/tuning/configMotionCruiseVelocity", maxVelocity));
    }

    public void zero() {
        climberMotor.setSelectedSensorPosition(0);
    }

    @Override
    public void periodic() {
        SmartDashboard.putNumber("climber/position", getPositionMeters());
        SmartDashboard.putNumber("climber/height voltage", getGroundSensorVoltage());
        SmartDashboard.putNumber("climber/velocity", climberMotor.getSelectedSensorVelocity());
        SmartDashboard.putNumber("climber/error", climberMotor.getClosedLoopError());
        SmartDashboard.putNumber("climber/current", climberMotor.getStatorCurrent());
        SmartDashboard.putNumber("climber/ratchetPosition", ratchetServo.get());
        SmartDashboard.putNumber("climber/throttle", climberMotor.getMotorOutputPercent());

    }

    public void configSoftLimitMeters(double min, double max) {
        climberMotor.configForwardSoftLimitThreshold((int) climberMetersToTicks(min), (int) climberMetersToTicks(max));
        climberMotor.configReverseSoftLimitThreshold((int) climberMetersToTicks(min), (int) climberMetersToTicks(max));
    }

    public void setPercent(double percent) {
        climberMotor.set(ControlMode.PercentOutput, percent);
    }

    public void stop() {
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

    public double getCurrent() {
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

    public double getGroundSensorVoltage() {
        return distanceSensor.getVoltage();
    }

    public enum RatchetState {
        ENGAGED(0),
        DISENGAGED(0.472);

        private double servoPosition;

        RatchetState(double servoPosition) {
            this.servoPosition = servoPosition;
        }
    }

    public void setRatchet(RatchetState state) {
        climberMotor.configPeakOutputForward(state == RatchetState.ENGAGED ? 0 : 1);
        ratchetServo.set(state.servoPosition);
        SmartDashboard.putBoolean("climber/ratchetEngaged", state == RatchetState.ENGAGED);
    }

    public Command commandRatchet(RatchetState state) {
        return new InstantCommand(() -> setRatchet(state));
    }

    public void setRatchet(double position) {
        ratchetServo.set(position);
    }

    public void setBrake(NeutralMode state) {
        climberMotor.setNeutralMode(state);
    }

    public Command commandStop() {
        return new InstantCommand(this::stop, this);
    }
}
