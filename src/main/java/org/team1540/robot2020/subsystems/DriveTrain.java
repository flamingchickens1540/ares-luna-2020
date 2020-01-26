package org.team1540.robot2020.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.BaseMotorController;
import com.ctre.phoenix.motorcontrol.can.TalonFX;
import edu.wpi.first.wpilibj.SPI.Port;
import edu.wpi.first.wpilibj.geometry.Pose2d;
import edu.wpi.first.wpilibj.geometry.Rotation2d;
import edu.wpi.first.wpilibj.geometry.Translation2d;
import edu.wpi.first.wpilibj.kinematics.DifferentialDriveOdometry;
import edu.wpi.first.wpilibj.kinematics.DifferentialDriveWheelSpeeds;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import org.team1540.robot2020.utils.CTREBaseMotorControllerEncoder;
import org.team1540.robot2020.utils.Encoder;
import org.team1540.robot2020.utils.NavX;

public class DriveTrain extends SubsystemBase {

    public static final double trackwidthMeters = 0.761388065;

    public static final double encoderMetersPerRev = 0.4863499587;
    public static final double encoderTicksPerRev = 512;
    public static final double encoderMetersPerTick = encoderMetersPerRev / encoderTicksPerRev;

    public static final int DRIVE_POSITION_SLOT_IDX = 0;
    public static final int DRIVE_VELOCITY_SLOT_IDX = 1;

    private TalonFX[] driveMotorAll;
    private TalonFX[] driveMotorMasters;
    private TalonFX[] driveMotorFollowers;
    private TalonFX[] driveMotorLefts;
    private TalonFX[] driveMotorRights;

    private TalonFX driveMotorLeftA = new TalonFX(0);
    private TalonFX driveMotorLeftB = new TalonFX(1);

    private TalonFX driveMotorRightA = new TalonFX(2);
    private TalonFX driveMotorRightB = new TalonFX(3);

    private Encoder leftEncoder = new CTREBaseMotorControllerEncoder(driveMotorLeftA, encoderMetersPerTick, false);
    private Encoder rightEncoder = new CTREBaseMotorControllerEncoder(driveMotorRightA, encoderMetersPerTick, false);

    private final NavX navx = new NavX(Port.kMXP);

    private double navxOffset = 0;

    private final DifferentialDriveOdometry odometry = new DifferentialDriveOdometry(new Rotation2d(getHeading()));
    private int saturationVoltage = 12;

    public DriveTrain() {
        initMotors();
        initEncoders();
    }

    private void initMotors() {
        driveMotorAll = new TalonFX[]{driveMotorLeftA, driveMotorLeftB, driveMotorRightA, driveMotorRightB};
        driveMotorMasters = new TalonFX[]{driveMotorLeftA, driveMotorRightA};
        driveMotorFollowers = new TalonFX[]{driveMotorLeftB, driveMotorRightB};
        driveMotorLefts = new TalonFX[]{driveMotorLeftA, driveMotorLeftB};
        driveMotorRights = new TalonFX[]{driveMotorRightA, driveMotorRightB};

        // TODO: Use TalonFXConfiguration instead

        for (BaseMotorController controller : driveMotorAll) {
            controller.configFactoryDefault();

            controller.setNeutralMode(NeutralMode.Brake);

            controller.configVoltageCompSaturation(saturationVoltage);
            controller.enableVoltageCompensation(true);
            controller.configPeakOutputForward(1);
            controller.configPeakOutputReverse(-1);

            controller.configOpenloopRamp(0);
            controller.configForwardSoftLimitEnable(false);
            controller.configReverseSoftLimitEnable(false);
            controller.overrideLimitSwitchesEnable(false);
        }

        for (TalonFX controller : driveMotorMasters) {
            // Position
            controller.config_kP(DRIVE_POSITION_SLOT_IDX, 0);
            controller.config_kI(DRIVE_POSITION_SLOT_IDX, 0);
            controller.config_kD(DRIVE_POSITION_SLOT_IDX, 0);
            controller.config_kF(DRIVE_POSITION_SLOT_IDX, 0);

            // Velocity
            controller.config_kP(DRIVE_VELOCITY_SLOT_IDX, 3);
            controller.config_kI(DRIVE_VELOCITY_SLOT_IDX, 0.02);
            controller.config_kF(DRIVE_VELOCITY_SLOT_IDX, 0);
            controller.config_kD(DRIVE_VELOCITY_SLOT_IDX, 0);
        }

        for (BaseMotorController talon : driveMotorLefts) {
            talon.setInverted(true);
        }

        for (BaseMotorController talon : driveMotorRights) {
            talon.setInverted(false);
        }

        driveMotorLeftA.setSensorPhase(false);
        driveMotorRightA.setSensorPhase(false);

        driveMotorLeftB.follow(driveMotorLeftA);

        driveMotorRightB.follow(driveMotorRightA);
    }

    private void initEncoders() {
        resetEncoders();
    }

    @Override
    public void periodic() {
        odometry.update(new Rotation2d(getHeading()), leftEncoder.getDistance(),
            rightEncoder.getDistance());

        SmartDashboard.putNumber("drive/encoderDistanceLeft", leftEncoder.getDistance());
        SmartDashboard.putNumber("drive/encoderDistanceRight", rightEncoder.getDistance());

        SmartDashboard.putNumber("drive/encoderSpeedLeft", getWheelSpeeds().leftMetersPerSecond);
        SmartDashboard.putNumber("drive/encoderSpeedRight", getWheelSpeeds().rightMetersPerSecond);

        SmartDashboard.putNumber("drive/encoderTicksLeft", driveMotorLeftA.getSelectedSensorPosition());
        SmartDashboard.putNumber("drive/encoderTicksRight", driveMotorRightA.getSelectedSensorPosition());

        Pose2d poseMeters = odometry.getPoseMeters();
        Translation2d postTranslation = poseMeters.getTranslation();
        SmartDashboard.putNumber("drive/odometry/X", postTranslation.getX());
        SmartDashboard.putNumber("drive/odometry/Y", postTranslation.getY());
        SmartDashboard.putNumber("drive/odometry/rotationDegrees", poseMeters.getRotation().getDegrees());
    }

    public Pose2d getPose() {
        return odometry.getPoseMeters();
    }

    public DifferentialDriveWheelSpeeds getWheelSpeeds() {
        return new DifferentialDriveWheelSpeeds(leftEncoder.getRate(), rightEncoder.getRate());
    }

    public void resetOdometry(Pose2d pose) {
        resetEncoders();
        odometry.resetPosition(pose, new Rotation2d(getHeading()));
    }

    public void tankDriveVolts(double leftVolts, double rightVolts) {
        driveMotorLeftA.set(ControlMode.PercentOutput, leftVolts / saturationVoltage);
        driveMotorRightA.set(ControlMode.PercentOutput, rightVolts / saturationVoltage);
    }

    public void tankDriveVelocity(double leftMetersPerSecond, double rightMetersPerSecond) {
        driveMotorLeftA.selectProfileSlot(DRIVE_VELOCITY_SLOT_IDX, 0);
        driveMotorRightA.selectProfileSlot(DRIVE_VELOCITY_SLOT_IDX, 0);

        driveMotorLeftA.set(ControlMode.Velocity, leftMetersPerSecond / encoderMetersPerTick / 10);
        driveMotorRightA.set(ControlMode.Velocity, rightMetersPerSecond / encoderMetersPerTick / 10);
    }

    public void tankDrivePercent(double leftSpeed, double rightSpeed) {
        driveMotorLeftA.set(ControlMode.PercentOutput, leftSpeed);
        driveMotorRightA.set(ControlMode.PercentOutput, rightSpeed);
    }

    public void resetEncoders() {
        leftEncoder.reset();
        rightEncoder.reset();
    }

    public double getHeading() {
        return navx.getAngleRadians();
    }

    public void zeroNavx() {
        navxOffset = navx.getYawRadians();
    }

    public double getNavxOffset() {
        return navxOffset;
    }
}
