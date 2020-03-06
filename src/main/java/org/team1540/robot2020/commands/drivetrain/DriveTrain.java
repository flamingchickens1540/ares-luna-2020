package org.team1540.robot2020.commands.drivetrain;

import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.StatorCurrentLimitConfiguration;
import edu.wpi.first.wpilibj.kinematics.DifferentialDriveWheelSpeeds;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import org.team1540.robot2020.utils.ChickenTalonFX;
import org.team1540.robot2020.utils.MotorConfigUtils;

public class DriveTrain extends SubsystemBase {

    public static final double kTrackwidthMeters = .706032166; //0.761388065;

    private final double drivetrainTicksPerMeter = 49866;

    private ChickenTalonFX driveMotorLeftA = new ChickenTalonFX(1, drivetrainTicksPerMeter, MotorConfigUtils.POSITION_SLOT_IDX, MotorConfigUtils.VELOCITY_SLOT_IDX);
    private ChickenTalonFX driveMotorLeftB = new ChickenTalonFX(2, drivetrainTicksPerMeter, MotorConfigUtils.POSITION_SLOT_IDX, MotorConfigUtils.VELOCITY_SLOT_IDX);

    private ChickenTalonFX driveMotorRightA = new ChickenTalonFX(3, drivetrainTicksPerMeter, MotorConfigUtils.POSITION_SLOT_IDX, MotorConfigUtils.VELOCITY_SLOT_IDX);
    private ChickenTalonFX driveMotorRightB = new ChickenTalonFX(4, drivetrainTicksPerMeter, MotorConfigUtils.POSITION_SLOT_IDX, MotorConfigUtils.VELOCITY_SLOT_IDX);

    private ChickenTalonFX[] driveMotorAll = new ChickenTalonFX[]{driveMotorLeftA, driveMotorLeftB, driveMotorRightA, driveMotorRightB};
    private ChickenTalonFX[] driveMotorLefts = new ChickenTalonFX[]{driveMotorLeftA, driveMotorLeftB};
    private ChickenTalonFX[] driveMotorRights = new ChickenTalonFX[]{driveMotorRightA, driveMotorRightB};


    public DriveTrain() {
        initMotors();
    }

    private void initMotors() {
        for (ChickenTalonFX talon : driveMotorAll) {
            MotorConfigUtils.setDefaultTalonFXConfig(talon);

            talon.configNeutralDeadband(0.01);

            talon.configStatorCurrentLimit(new StatorCurrentLimitConfiguration(true, 90, 0, 0), 50);
            talon.enableVoltageCompensation(true);
            talon.configVoltageCompSaturation(10);
        }

        for (ChickenTalonFX talon : driveMotorLefts) {
            talon.setInverted(false);
        }

        for (ChickenTalonFX talon : driveMotorRights) {
            talon.setInverted(true);
        }

        driveMotorLeftB.follow(driveMotorLeftA);
        driveMotorRightB.follow(driveMotorRightA);

        setBrakes(NeutralMode.Coast);
    }

    public void setBrakes(NeutralMode brake) {
        for (ChickenTalonFX talon : driveMotorAll) {
            talon.setNeutralMode(brake);
        }
    }

    @Override
    public void periodic() {
        logState();
    }

    private void logState() {
        SmartDashboard.putNumber("driveTrain/encoderDistanceLeft", driveMotorLeftA.getDistanceMeters());
        SmartDashboard.putNumber("driveTrain/encoderDistanceRight", driveMotorRightA.getDistanceMeters());

        SmartDashboard.putNumber("driveTrain/encoderSpeedLeft", getWheelSpeeds().leftMetersPerSecond);
        SmartDashboard.putNumber("driveTrain/encoderSpeedRight", getWheelSpeeds().rightMetersPerSecond);

        SmartDashboard.putNumber("driveTrain/encoderTicksLeft", driveMotorLeftA.getSelectedSensorPosition());
        SmartDashboard.putNumber("driveTrain/encoderTicksRight", driveMotorRightA.getSelectedSensorPosition());
    }

    public DifferentialDriveWheelSpeeds getWheelSpeeds() {
        return new DifferentialDriveWheelSpeeds(driveMotorLeftA.getRateMetersPerSecond(), driveMotorRightA.getRateMetersPerSecond());
    }

    public void setVoltage(double leftVolts, double rightVolts) {
        driveMotorLeftA.setVoltage(leftVolts);
        driveMotorRightA.setVoltage(rightVolts);
    }

    public void setVelocityMetersPerSecond(double leftMetersPerSecond, double rightMetersPerSecond) {
        driveMotorLeftA.setVelocityMetersPerSecond(leftMetersPerSecond);
        driveMotorRightA.setVelocityMetersPerSecond(rightMetersPerSecond);
    }

    public void setPercent(double leftPercent, double rightPercent) {
        driveMotorLeftA.setPercent(leftPercent);
        driveMotorRightA.setPercent(rightPercent);
    }

    public double getDistanceLeft() {
        return driveMotorLeftA.getDistanceMeters();
    }

    public double getDistanceRight() {
        return driveMotorRightA.getDistanceMeters();
    }

    public void resetEncoders() {
        driveMotorLeftA.setSelectedSensorPosition(0);
        driveMotorRightA.setSelectedSensorPosition(0);
    }

    public Command commandStop() {
        return new InstantCommand(this::disableMotors, this);
    }

    public void disableMotors() {
        setPercent(0, 0);
    }
}
