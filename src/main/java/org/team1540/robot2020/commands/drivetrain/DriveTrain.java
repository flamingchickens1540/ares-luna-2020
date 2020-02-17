package org.team1540.robot2020.commands.drivetrain;

import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.StatorCurrentLimitConfiguration;
import edu.wpi.first.wpilibj.geometry.Pose2d;
import edu.wpi.first.wpilibj.geometry.Rotation2d;
import edu.wpi.first.wpilibj.geometry.Translation2d;
import edu.wpi.first.wpilibj.kinematics.DifferentialDriveOdometry;
import edu.wpi.first.wpilibj.kinematics.DifferentialDriveWheelSpeeds;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import org.team1540.robot2020.utils.ChickenTalonFX;
import org.team1540.robot2020.utils.MotorConfigUtils;
import org.team1540.robot2020.utils.NavX;

public class DriveTrain extends SubsystemBase {

    public static final double kTrackwidthMeters = 0.761388065;

    private final double drivetrainTicksPerMeter = 1052.7398858397396;

    private ChickenTalonFX driveMotorLeftA = new ChickenTalonFX(1, drivetrainTicksPerMeter, MotorConfigUtils.POSITION_SLOT_IDX, MotorConfigUtils.VELOCITY_SLOT_IDX);
    private ChickenTalonFX driveMotorLeftB = new ChickenTalonFX(2, drivetrainTicksPerMeter, MotorConfigUtils.POSITION_SLOT_IDX, MotorConfigUtils.VELOCITY_SLOT_IDX);

    private ChickenTalonFX driveMotorRightA = new ChickenTalonFX(3, drivetrainTicksPerMeter, MotorConfigUtils.POSITION_SLOT_IDX, MotorConfigUtils.VELOCITY_SLOT_IDX);
    private ChickenTalonFX driveMotorRightB = new ChickenTalonFX(4, drivetrainTicksPerMeter, MotorConfigUtils.POSITION_SLOT_IDX, MotorConfigUtils.VELOCITY_SLOT_IDX);

    private ChickenTalonFX[] driveMotorAll = new ChickenTalonFX[]{driveMotorLeftA, driveMotorLeftB, driveMotorRightA, driveMotorRightB};
    private ChickenTalonFX[] driveMotorLefts = new ChickenTalonFX[]{driveMotorLeftA, driveMotorLeftB};
    private ChickenTalonFX[] driveMotorRights = new ChickenTalonFX[]{driveMotorRightA, driveMotorRightB};

    private NavX navx;

    private double navxOffset = 0;

    // TODO need a wrapper for the odometry class that allows us to not reset the encoders-  should store its own relative offsets

    private final DifferentialDriveOdometry odometry;

    public DriveTrain(NavX navx) {
        initMotors();
        this.navx = navx;
        odometry = new DifferentialDriveOdometry(Rotation2d.fromDegrees(getHeading()));
    }

    private void initMotors() {
        for (ChickenTalonFX talon : driveMotorAll) {
            MotorConfigUtils.setDefaultTalonFXConfig(talon);

            talon.configNeutralDeadband(0.01);

            talon.configStatorCurrentLimit(new StatorCurrentLimitConfiguration(true, 90, 0, 0), 50);
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
        odometry.update(Rotation2d.fromDegrees(getHeading()), driveMotorLeftA.getDistanceMeters(), driveMotorRightA.getDistanceMeters());

        logState();
    }

    private void logState() {
        SmartDashboard.putNumber("driveTrain/encoderDistanceLeft", driveMotorLeftA.getDistanceMeters());
        SmartDashboard.putNumber("driveTrain/encoderDistanceRight", driveMotorRightA.getDistanceMeters());

        SmartDashboard.putNumber("driveTrain/encoderSpeedLeft", getWheelSpeeds().leftMetersPerSecond);
        SmartDashboard.putNumber("driveTrain/encoderSpeedRight", getWheelSpeeds().rightMetersPerSecond);

        Pose2d poseMeters = odometry.getPoseMeters();
        Translation2d poseTranslation = poseMeters.getTranslation();
        SmartDashboard.putNumber("driveTrain/odometry/X", poseTranslation.getX());
        SmartDashboard.putNumber("driveTrain/odometry/Y", poseTranslation.getY());
        SmartDashboard.putNumber("driveTrain/odometry/rotationDegrees", poseMeters.getRotation().getDegrees());
    }

    public Pose2d getPose() {
        return odometry.getPoseMeters();
    }

    public DifferentialDriveWheelSpeeds getWheelSpeeds() {
        return new DifferentialDriveWheelSpeeds(driveMotorLeftA.getRateMetersPerSecond(), driveMotorRightA.getRateMetersPerSecond());
    }

    public void resetOdometry(Pose2d pose) {
        odometry.resetPosition(pose, Rotation2d.fromDegrees(getHeading()));
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
        return driveMotorRightA.getDistanceMeters();
    }

    public double getDistanceRight() {
        return driveMotorRightA.getDistanceMeters();
    }

    public double getHeading() {
        return Math.IEEEremainder(navx.getYawRadians() + navxOffset, 360);
    }
}
