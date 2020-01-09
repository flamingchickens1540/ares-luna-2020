package org.team1540.robot2020.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.BaseMotorController;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import com.ctre.phoenix.motorcontrol.can.VictorSPX;
import edu.wpi.first.wpilibj.SPI.Port;
import edu.wpi.first.wpilibj.geometry.Pose2d;
import edu.wpi.first.wpilibj.geometry.Rotation2d;
import edu.wpi.first.wpilibj.geometry.Translation2d;
import edu.wpi.first.wpilibj.kinematics.DifferentialDriveKinematics;
import edu.wpi.first.wpilibj.kinematics.DifferentialDriveOdometry;
import edu.wpi.first.wpilibj.kinematics.DifferentialDriveWheelSpeeds;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import org.team1540.robot2020.shouldbeinrooster.Encoder;
import org.team1540.robot2020.shouldbeinrooster.NavX;
import org.team1540.robot2020.shouldbeinrooster.TalonEncoder;

public class DriveTrain extends SubsystemBase {

    // Feed forward constants
    public final double ksVolts = 0.669;
    public final double kvVoltSecondsPerMeter = 2.76;
    public final double kaVoltSecondsSquaredPerMeter = 0.662;

    // Ramsete PID controllers
//    public final double kPDriveVel = 19.3;
    public final double kPDriveVel = 1;
//    public final double kPDriveVel = 0;

    private final double kTrackwidthMeters = 0.761388065;
    public final DifferentialDriveKinematics kDriveKinematics =
        new DifferentialDriveKinematics(kTrackwidthMeters);

    private final double kWheelCircumference = 0.4863499587;
    private final double kEncoderPPR = 512;
    private final double encoderMetersPerTick = kWheelCircumference / kEncoderPPR;

    // Motion control
    public final double kRamseteB = 2;
    public final double kRamseteZeta = 0.7;

    public final double kMaxSpeedMetersPerSecond = 2;
    public final double kMaxAccelerationMetersPerSecondSquared = 1;

    private static final int DRIVE_POSITION_SLOT_IDX = 0;
    private static final int DRIVE_VELOCITY_SLOT_IDX = 1;

    private BaseMotorController[] driveMotorAll;
    private TalonSRX[] driveMotorMasters;
    private VictorSPX[] driveMotorFollowers;
    private BaseMotorController[] driveMotorLeft;
    private BaseMotorController[] driveMotorRight;

    private TalonSRX driveMotorLeftA = new TalonSRX(13);
    private VictorSPX driveMotorLeftB = new VictorSPX(12);
    private VictorSPX driveMotorLeftC = new VictorSPX(11);

    private TalonSRX driveMotorRightA = new TalonSRX(1);
    private VictorSPX driveMotorRightC = new VictorSPX(3);
    private VictorSPX driveMotorRightB = new VictorSPX(2);

    private Encoder leftEncoder = new TalonEncoder(driveMotorLeftA);
    private Encoder rightEncoder = new TalonEncoder(driveMotorRightA);

    private final NavX navx = new NavX(Port.kMXP);

    private double navxOffset = 0;

    private final DifferentialDriveOdometry odometry = new DifferentialDriveOdometry(Rotation2d.fromDegrees(getHeading()));
    private int saturationVoltage = 12;

    public DriveTrain() {
        initMotors();
        initEncoders();
    }

    private void initMotors() {
        driveMotorAll = new BaseMotorController[]{driveMotorLeftA, driveMotorLeftB, driveMotorLeftC, driveMotorRightA, driveMotorRightB, driveMotorRightC};
        driveMotorMasters = new TalonSRX[]{driveMotorLeftA, driveMotorRightA};
        driveMotorFollowers = new VictorSPX[]{driveMotorLeftB, driveMotorLeftC, driveMotorRightB, driveMotorRightC};
        driveMotorLeft = new BaseMotorController[]{driveMotorLeftA, driveMotorLeftB, driveMotorLeftC};
        driveMotorRight = new BaseMotorController[]{driveMotorRightA, driveMotorRightB, driveMotorRightC};

        for (BaseMotorController controller : driveMotorAll) {
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

        for (TalonSRX controller : driveMotorMasters) {
            controller.setNeutralMode(NeutralMode.Brake);

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

            controller.configPeakCurrentLimit(0);
            controller.configContinuousCurrentLimit(40);
        }

        for (BaseMotorController talon : driveMotorLeft) {
            talon.setInverted(true);
        }

        for (BaseMotorController talon : driveMotorRight) {
            talon.setInverted(false);
        }

        driveMotorLeftA.setSensorPhase(false);
        driveMotorRightA.setSensorPhase(false);

        driveMotorLeftB.follow(driveMotorLeftA);
        driveMotorLeftC.follow(driveMotorLeftA);

        driveMotorRightB.follow(driveMotorRightA);
        driveMotorRightC.follow(driveMotorRightA);
    }

    private void initEncoders() {
        leftEncoder.setDistancePerPulse(encoderMetersPerTick);
        rightEncoder.setDistancePerPulse(encoderMetersPerTick);

        leftEncoder.setInverted(false);
        rightEncoder.setInverted(false);

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

    public void tankDrivePercent(double leftSpeed, double rightSpeed) {
        driveMotorLeftA.set(ControlMode.PercentOutput, leftSpeed);
        driveMotorRightA.set(ControlMode.PercentOutput, rightSpeed);
    }

    public void resetEncoders() {
        driveMotorLeftA.setSelectedSensorPosition(0);
        driveMotorRightA.setSelectedSensorPosition(0);
    }

    public double getHeading() {
        return Math.IEEEremainder(navx.getAngle(), 360);
    }

    public void zeroNavx() {
        navxOffset = navx.getYawRadians();
    }

    public double getNavxOffset() {
        return navxOffset;
    }
}
