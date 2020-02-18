package org.team1540.robot2020;

import edu.wpi.first.wpilibj.I2C;
import edu.wpi.first.wpilibj.SPI;
import edu.wpi.first.wpilibj.geometry.Pose2d;
import edu.wpi.first.wpilibj.geometry.Rotation2d;
import edu.wpi.first.wpilibj.geometry.Translation2d;
import edu.wpi.first.wpilibj.kinematics.DifferentialDriveOdometry;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import org.team1540.robot2020.commands.drivetrain.DriveTrain;
import org.team1540.robot2020.utils.LIDARLite;
import org.team1540.robot2020.utils.Limelight;
import org.team1540.robot2020.utils.NavX;
import org.team1540.rooster.datastructures.threed.Transform3D;
import org.team1540.rooster.datastructures.utils.RotationUtils;


public class LocalizationManager extends CommandBase {

    private final double TARGET_HEIGHT = 230.0; // cm
    private final double LIMELIGHT_HEIGHT = 93.98; // cm
    private final double LIMELIGHT_ANGLE = Math.toRadians(23);
    private final double LIDAR_HEIGHT = 111.125; // cm
    private final double LIDAR_ANGLE = Math.toRadians(4.5);
    private final double THE_LIDAR_VS_LIMELIGHT_ANGLE_SELECTION_TOLERANCE = 20;
    private final double NAVX_OFFSET = 0;

    private final DifferentialDriveOdometry odometry;
    private Limelight limelight = new Limelight("limelight");
    private LIDARLite lidar = new LIDARLite(I2C.Port.kOnboard);
    private NavX navx = new NavX(SPI.Port.kMXP);
    private DriveTrain driveTrain;

    public LocalizationManager(DriveTrain driveTrain) {
        this.driveTrain = driveTrain;
        odometry = new DifferentialDriveOdometry(Rotation2d.fromDegrees(getHeading()));
    }

    public Transform3D getRobotToHexagonTransform(double distance) {
        double angle = limelight.getTargetAngles().getX();
        return new Transform3D(Math.cos(angle) * distance, Math.sin(angle) * distance, 249, 0, 0, -navx.getYawRadians() - angle);
    }

    public Transform3D getRobotToRearHoleTransform() {
        Transform3D robotToOdom = new Transform3D(odometry.getPoseMeters().getTranslation().getX(), odometry.getPoseMeters().getTranslation().getY(), Math.toRadians(odometry.getPoseMeters().getRotation().getDegrees()));
        return getRobotToHexagonTransform(getDistance()).add(robotToOdom);
    }

    public void initialize() {
        lidar.startMeasuring();
    }

    public void setLimelightLeds(boolean state) {
        limelight.setLeds(state);
    }

    private boolean useLimelight() {
        return limelight.isTargetFound() && Math.abs(limelight.getTargetAngles().getX()) < THE_LIDAR_VS_LIMELIGHT_ANGLE_SELECTION_TOLERANCE;
    }

    public double getDistance() {
        if (useLimelight()) {
            return (TARGET_HEIGHT - LIMELIGHT_HEIGHT) / Math.tan(LIMELIGHT_ANGLE + limelight.getTargetAngles().getY());
        } else {
            return (TARGET_HEIGHT - LIDAR_HEIGHT) / Math.tan(LIDAR_ANGLE + lidar.getDistance());
        }
    }

    @Override
    public void execute() {
        Transform3D robotToTarget = getRobotToHexagonTransform(lidar.getDistance());

        SmartDashboard.putNumber("telemetry/lidarDistance", lidar.getDistance());
        SmartDashboard.putNumber("telemetry/angleRadians", navx.getAngleRadians());
        SmartDashboard.putNumber("telemetry/yawRadians", navx.getYawRadians());

        SmartDashboard.putNumber("transform/robotToTarget/X", robotToTarget.getPosition().getX());
        SmartDashboard.putNumber("transform/robotToTarget/Y", robotToTarget.getPosition().getY());
        SmartDashboard.putNumber("transform/robotToTarget/Z", robotToTarget.getPosition().getZ());
        SmartDashboard.putNumber("transform/robotToTarget/", RotationUtils.getRPYVec(robotToTarget.getOrientation()).getX());
        SmartDashboard.putNumber("transform/robotToTarget/", RotationUtils.getRPYVec(robotToTarget.getOrientation()).getY());
        SmartDashboard.putNumber("transform/robotToTarget/", RotationUtils.getRPYVec(robotToTarget.getOrientation()).getZ());

        SmartDashboard.putBoolean("transform/usingLimelight", useLimelight());

        Pose2d poseMeters = odometry.getPoseMeters();
        Translation2d poseTranslation = poseMeters.getTranslation();
        SmartDashboard.putNumber("driveTrain/odometry/X", poseTranslation.getX());
        SmartDashboard.putNumber("driveTrain/odometry/Y", poseTranslation.getY());
        SmartDashboard.putNumber("driveTrain/odometry/rotationDegrees", poseMeters.getRotation().getDegrees());

        odometry.update(Rotation2d.fromDegrees(getHeading()), driveTrain.getDistanceLeft(), driveTrain.getDistanceRight());
    }

    @Override
    public boolean runsWhenDisabled() {
        return true;
    }

    public void resetOdometry(Pose2d pose) {
        odometry.resetPosition(pose, Rotation2d.fromDegrees(getHeading()));
    }

    public double getHeading() {
        return Math.IEEEremainder(navx.getYawRadians() + NAVX_OFFSET, 360);
    }

    public Pose2d getPose() {
        return odometry.getPoseMeters();
    }
}
