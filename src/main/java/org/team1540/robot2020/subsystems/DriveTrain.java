package org.team1540.robot2020.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.TalonFX;
import edu.wpi.first.wpilibj.SPI.Port;
import edu.wpi.first.wpilibj.geometry.Pose2d;
import edu.wpi.first.wpilibj.geometry.Rotation2d;
import edu.wpi.first.wpilibj.geometry.Translation2d;
import edu.wpi.first.wpilibj.kinematics.DifferentialDriveKinematics;
import edu.wpi.first.wpilibj.kinematics.DifferentialDriveOdometry;
import edu.wpi.first.wpilibj.kinematics.DifferentialDriveWheelSpeeds;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import org.team1540.robot2020.utils.CTREBaseMotorControllerEncoder;
import org.team1540.robot2020.utils.Encoder;
import org.team1540.robot2020.utils.NavX;

public class DriveTrain extends SubsystemBase {

    // Feed forward constants
    public static final double ksVolts = 0.669;
    public static final double kvVoltSecondsPerMeter = 2.76;
    public static final double kaVoltSecondsSquaredPerMeter = 0.662;

    // Ramsete PID controllers
//    public static final double kPDriveVel = 19.3;
    public static final double kPDriveVel = 1;
//    public static final double kPDriveVel = 0;

    public static final double kTrackwidthMeters = 0.761388065;
    public final DifferentialDriveKinematics kDriveKinematics =
        new DifferentialDriveKinematics(kTrackwidthMeters);

    private final double kWheelCircumference = 0.4863499587;
    private final double kEncoderPPR = 512;
    private final double encoderMetersPerTick = kWheelCircumference / kEncoderPPR;

    // Motion control
    public static final double kRamseteB = 2;
    public static final double kRamseteZeta = 0.7;

    public static final double kMaxSpeedMetersPerSecond = 2;
    public static final double kMaxAccelerationMetersPerSecondSquared = 1;

    private static final int DRIVE_POSITION_SLOT_IDX = 0;
    private static final int DRIVE_VELOCITY_SLOT_IDX = 1;

    private TalonFX[] driveMotorAll;
    private TalonFX[] driveMotorMasters;
    private TalonFX[] driveMotorFollowers;
    private TalonFX[] driveMotorLefts;
    private TalonFX[] driveMotorRights;

    private TalonFX driveMotorLeftA = new TalonFX(0);
    private TalonFX driveMotorLeftB = new TalonFX(1);

    private TalonFX driveMotorRightA = new TalonFX(2);
    private TalonFX driveMotorRightB = new TalonFX(3);

    private Encoder leftEncoder = new CTREBaseMotorControllerEncoder(driveMotorLeftA, encoderMetersPerTick, true);
    private Encoder rightEncoder = new CTREBaseMotorControllerEncoder(driveMotorRightA, encoderMetersPerTick, false);

    public final NavX navx = new NavX(Port.kMXP);

    private double navxOffset = 0;

    private final DifferentialDriveOdometry odometry = new DifferentialDriveOdometry(Rotation2d.fromDegrees(getHeading()));
    private int saturationVoltage = 12;

    public DriveTrain() {
        initMotors();
        initEncoders();
    }

    private void initMotors() {
        driveMotorAll = new TalonFX[]{driveMotorLeftA, driveMotorLeftB, driveMotorRightA, driveMotorRightB};
        driveMotorMasters = new TalonFX[]{driveMotorLeftA, driveMotorRightA};
        driveMotorFollowers = new TalonFX[]{driveMotorRightB, driveMotorLeftB};
        driveMotorLefts = new TalonFX[]{driveMotorLeftA, driveMotorLeftB};
        driveMotorRights = new TalonFX[]{driveMotorRightA, driveMotorRightB};

        // TODO: Use TalonFXConfiguration instead

        for (TalonFX talon : driveMotorAll) {
            // TODO this should restoreFactoryDefaults() on all motors

            talon.setNeutralMode(NeutralMode.Brake);

            talon.configVoltageCompSaturation(saturationVoltage);
            talon.enableVoltageCompensation(true);
            talon.configPeakOutputForward(1);
            talon.configPeakOutputReverse(-1);

            talon.configOpenloopRamp(0);
            talon.configForwardSoftLimitEnable(false);
            talon.configReverseSoftLimitEnable(false);
            talon.overrideLimitSwitchesEnable(false);
        }

        for (TalonFX talon : driveMotorMasters) {
            talon.setNeutralMode(NeutralMode.Brake);

            // Position
            talon.config_kP(DRIVE_POSITION_SLOT_IDX, 0);
            talon.config_kI(DRIVE_POSITION_SLOT_IDX, 0);
            talon.config_kD(DRIVE_POSITION_SLOT_IDX, 0);
            talon.config_kF(DRIVE_POSITION_SLOT_IDX, 0);

            // Velocity
            talon.config_kP(DRIVE_VELOCITY_SLOT_IDX, 3);
            talon.config_kI(DRIVE_VELOCITY_SLOT_IDX, 0.02);
            talon.config_kF(DRIVE_VELOCITY_SLOT_IDX, 0);
            talon.config_kD(DRIVE_VELOCITY_SLOT_IDX, 0);
        }

        for (TalonFX talon : driveMotorLefts) {
            talon.setInverted(true);
        }

        for (TalonFX talon : driveMotorRights) {
            talon.setInverted(false);
        }

        driveMotorLeftA.setSensorPhase(false);
        driveMotorRightA.setSensorPhase(false);

//            talon.configPeakCurrentLimit(0);
//            talon.configContinuousCurrentLimit(40);

        driveMotorLeftB.follow(driveMotorLeftA);

        driveMotorRightB.follow(driveMotorRightA);
    }

    private void initEncoders() {
        resetEncoders();
    }

    @Override
    public void periodic() {
        odometry.update(Rotation2d.fromDegrees(getHeading()), leftEncoder.getDistance(),
            rightEncoder.getDistance());

        SmartDashboard.putNumber("drive/encoderDistanceLeft", leftEncoder.getDistance());
        SmartDashboard.putNumber("drive/encoderDistanceRight", rightEncoder.getDistance());

        SmartDashboard.putNumber("drive/encoderSpeedLeft", getWheelSpeeds().leftMetersPerSecond);
        SmartDashboard.putNumber("drive/encoderSpeedRight", getWheelSpeeds().rightMetersPerSecond);

        SmartDashboard.putNumber("drive/encoderTicksLeft", driveMotorLeftA.getSelectedSensorPosition());
        SmartDashboard.putNumber("drive/encoderTicksRight", driveMotorRightA.getSelectedSensorPosition());

        Pose2d poseMeters = odometry.getPoseMeters();
        Translation2d poseTranslation = poseMeters.getTranslation();
        SmartDashboard.putNumber("drive/odometry/X", poseTranslation.getX());
        SmartDashboard.putNumber("drive/odometry/Y", poseTranslation.getY());
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
        odometry.resetPosition(pose, Rotation2d.fromDegrees(getHeading()));
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

    public void tankDrivePercent(double leftPercent, double rightPercent) {
        driveMotorLeftA.set(ControlMode.PercentOutput, leftPercent);
        driveMotorRightA.set(ControlMode.PercentOutput, rightPercent);
    }

    public void resetEncoders() {
        leftEncoder.reset();
        rightEncoder.reset();
    }

    public double getHeading() {
        return Math.IEEEremainder(navx.getYawRadians() + navxOffset, 360);
    }

    public void zeroNavx() {
        navxOffset = navx.getYawRadians();
    }
}
