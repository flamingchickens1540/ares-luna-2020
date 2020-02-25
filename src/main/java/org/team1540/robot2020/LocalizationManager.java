package org.team1540.robot2020;

import edu.wpi.first.wpilibj.*;
import edu.wpi.first.wpilibj.geometry.Pose2d;
import edu.wpi.first.wpilibj.geometry.Rotation2d;
import edu.wpi.first.wpilibj.geometry.Translation2d;
import edu.wpi.first.wpilibj.kinematics.DifferentialDriveOdometry;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import org.apache.commons.math3.geometry.euclidean.threed.Vector3D;
import org.apache.commons.math3.geometry.euclidean.twod.Vector2D;
import org.jetbrains.annotations.NotNull;
import org.team1540.robot2020.commands.drivetrain.DriveTrain;
import org.team1540.robot2020.utils.InstCommand;
import org.team1540.robot2020.utils.LIDARLite;
import org.team1540.robot2020.utils.Limelight;
import org.team1540.robot2020.utils.NavX;
import org.team1540.rooster.datastructures.threed.Transform3D;
import org.team1540.rooster.datastructures.utils.RotationUtils;
import org.team1540.rooster.wrappers.RevBlinken;

import javax.annotation.Nullable;

import static edu.wpi.first.wpilibj2.command.CommandGroupBase.sequence;


public class LocalizationManager extends CommandBase {

    private final double HEXAGON_HEIGHT = 2.49;
    private final double VISION_TARGET_HEIGHT = 2.30;
    private final double LIMELIGHT_HEIGHT = 0.9398;
    private final double LIMELIGHT_PITCH = Math.toRadians(23);
    private final double LIDAR_PITCH = Math.toRadians(4.5);
    private final double LIDAR_SELECTION_TOLERANCE = Math.toRadians(3);
    private final double LIDAR_SELECTION_BUFFER = Math.toRadians(0.5);
    private final Transform3D HEXAGON_TO_INNER_PORT = new Transform3D(0.74295, 0, 0);
    private final DifferentialDriveOdometry odometry;
    private boolean useLidar = false;
    private boolean forceLimelightLEDOn = false;
    private Limelight limelight = new Limelight("limelight");
    private LIDARLite lidar = new LIDARLite(I2C.Port.kOnboard);
    private NavX navx = new NavX(SPI.Port.kMXP);
    private DriveTrain driveTrain;
    private RevBlinken blinken = new RevBlinken(7);

    private DigitalOutput buttonLed = new DigitalOutput(4);
    private DigitalInput buttonSignal = new DigitalInput(5);

    @Nullable
    private Transform3D odomToHexagon = null;
    private boolean acceptLimelight = true;

    public LocalizationManager(DriveTrain driveTrain) {
        this.driveTrain = driveTrain;

        // TODO: This is currently being done for the odometry-- remove this in future
        driveTrain.resetEncoders();
        navx.zeroYaw();

        odometry = new DifferentialDriveOdometry(new Rotation2d(navx.getYawRadians()));

        var button = new Trigger(buttonSignal::get);

        Notifier blinkNotifier = new Notifier(() -> buttonLed.set(!buttonLed.get()));
        blinkNotifier.startPeriodic(0.4);

        CommandBase zeroNavxCommand = sequence(
                new InstCommand(() -> {
                    blinkNotifier.stop();
                    blinkNotifier.startPeriodic(0.07);
                }, true),
                new WaitCommand(1),
                new InstCommand(() -> {
                    navx.zeroYaw();
                    System.out.println("NavX Zeroed!");
                    blinkNotifier.stop();
                    buttonLed.set(true);
                }, true)
        );
        button.whenActive(zeroNavxCommand);
        SmartDashboard.putData("localizationManager/zeroYaw", zeroNavxCommand);
    }

    public void initialize() {
        lidar.startMeasuring();
    }

    public void setLimelightLeds(boolean state) {
        limelight.setLeds(state);
    }

    private void updateOdomToHexagonTransform() {
        SmartDashboard.putBoolean("localizationManager/isAcceptingLimelight", acceptLimelight);
        if (acceptLimelight && isLimelightTargetFound()) {
            double robotToTargetDistance = getBestSensorDistance();
            double robotToTargetAngle = -limelight.getTargetAngles().getX();
            double targetAngleRelativeToRobot = -navx.getYawRadians();

            Transform3D robotToTarget = new Transform3D(Math.cos(robotToTargetAngle) * robotToTargetDistance, Math.sin(robotToTargetAngle) * robotToTargetDistance, HEXAGON_HEIGHT, 0, 0, targetAngleRelativeToRobot);
            odomToHexagon = getOdomToRobot().add(robotToTarget);
        }
    }

    public boolean isLimelightTargetFound() {
        return limelight.isTargetFound();
    }

    public void stopAcceptingLimelight() {
        acceptLimelight = false;
    }

    public void startAcceptingLimelight() {
        acceptLimelight = true;
    }

    @NotNull
    private Transform3D getOdomToRobot() {
        return new Transform3D(odometry.getPoseMeters().getTranslation().getX(), odometry.getPoseMeters().getTranslation().getY(), odometry.getPoseMeters().getRotation().getRadians());
    }

    public boolean useLidarForDistanceEst() {
        double limelightAngle = Math.abs(limelight.getTargetAngles().getX());

        // TODO: Make a util for thermostat control

        if (useLidar && (limelightAngle > LIDAR_SELECTION_TOLERANCE + LIDAR_SELECTION_BUFFER)) {
            useLidar = false;
        }

        if (!useLidar && (limelightAngle < LIDAR_SELECTION_TOLERANCE - LIDAR_SELECTION_BUFFER)) {
            useLidar = true;
        }

        return useLidar;
    }

    private double getBestSensorDistance() {
        return useLidarForDistanceEst() ? getCorrectedLidarDistance() : getLimelightDistance();
    }

    private double getLimelightDistance() {
        return (VISION_TARGET_HEIGHT - LIMELIGHT_HEIGHT) / Math.tan(LIMELIGHT_PITCH + limelight.getTargetAngles().getY());
    }

    public double getCorrectedLidarDistance() {
        return lidar.getDistance() * Math.cos(LIDAR_PITCH);
    }

    @Override
    public void execute() {
        updateOdometry();
        updateOdomToHexagonTransform();

        SmartDashboard.putNumber("localizationManager/angleRadians", navx.getAngleRadians());
        SmartDashboard.putNumber("localizationManager/yawRadians", navx.getYawRadians());
        SmartDashboard.putBoolean("localizationManager/isLimelightTargetFound", limelight.isTargetFound());
        SmartDashboard.putBoolean("localizationManager/robotToTargetDistanceUseLidar", useLidarForDistanceEst());
        SmartDashboard.putNumber("localizationManager/robotToTargetDistanceLimelight", getLimelightDistance());
        SmartDashboard.putNumber("localizationManager/robotToTargetDistanceLidar", getCorrectedLidarDistance());

        if (odomToHexagon != null) {
            putTransform(odomToHexagon, "localizationManager/odomToHexagon");
            putTransform(getRobotToRearHoleTransform(), "localizationManager/robotToRearHole");
        }

        Pose2d poseMeters = odometry.getPoseMeters();
        Translation2d poseTranslation = poseMeters.getTranslation();
        SmartDashboard.putNumber("localizationManager/odometry/X", poseTranslation.getX());
        SmartDashboard.putNumber("localizationManager/odometry/Y", poseTranslation.getY());
        SmartDashboard.putNumber("localizationManager/odometry/rotationDegrees", poseMeters.getRotation().getDegrees());

        setLimelightLEDs();
        setRobotLEDs();
    }

    public void putTransform(Transform3D transform3D, String prefix) {
        Vector3D position = transform3D.getPosition();
        SmartDashboard.putNumber(prefix + "/X", position.getX());
        SmartDashboard.putNumber(prefix + "/Y", position.getY());
        SmartDashboard.putNumber(prefix + "/Z", position.getZ());
        Vector3D rpyVec = RotationUtils.getRPYVec(transform3D.getOrientation());
        SmartDashboard.putNumber(prefix + "/roll", rpyVec.getX());
        SmartDashboard.putNumber(prefix + "/pitch", rpyVec.getY());
        SmartDashboard.putNumber(prefix + "/yaw", rpyVec.getZ());
        SmartDashboard.putNumber(prefix + "/xyMagnitude", new Vector2D(position.getX(), position.getY()).getNorm());
    }

    private void updateOdometry() {
        odometry.update(new Rotation2d(navx.getYawRadians()), driveTrain.getDistanceLeft(), driveTrain.getDistanceRight());
    }

    @Override
    public boolean runsWhenDisabled() {
        return true;
    }

    public void resetOdometry(Pose2d pose) {
        odometry.resetPosition(pose, new Rotation2d(navx.getYawRadians()));
    }

    public Pose2d odometryGetPose() {
        return odometry.getPoseMeters();
    }

    @Nullable
    public Transform3D getRobotToHexagonTransform() {
        if (odomToHexagon == null) return null;
        return getOdomToRobot().negate().add(odomToHexagon);
    }

    @Nullable
    public Transform3D getRobotToRearHoleTransform() {
        Transform3D robotToRearHoleTransform = getRobotToHexagonTransform();
        if (robotToRearHoleTransform == null) return null;
        return robotToRearHoleTransform.add(HEXAGON_TO_INNER_PORT);
    }

    public boolean shouldTargetInnerPort() {
        Transform3D robotToRearHoleTransform = getRobotToRearHoleTransform();
        if (robotToRearHoleTransform == null) return false;
        double angle = robotToRearHoleTransform.getOrientation().getAngle();
        SmartDashboard.putNumber("pointToTarget/targetSwitchingAngle", Math.toDegrees(angle));
        boolean targetingInnerPort = angle < Math.toRadians(20);
        SmartDashboard.putBoolean("pointToTarget/targetingInnerPort", targetingInnerPort);
        return targetingInnerPort;
    }

    public double getRate() {
        return navx.getRate();
    }

    public double getYawRadians() {
        return navx.getYawRadians();
    }

    public void setLimelightLEDs() {
        if (forceLimelightLEDOn) {
            setLimelightLeds(true);
        } else {
            if (getRobotToHexagonTransform() != null) {
                double x = getRobotToHexagonTransform().getPosition().getX();
                double y = getRobotToHexagonTransform().getPosition().getY();
                if (Math.abs(Math.atan2(y, x)) > Math.toRadians(40)) {
                    setLimelightLeds(false);
                } else {
                    setLimelightLeds(true);
                }
            } else { // If we dont a have a pose then fall back to keeping the LEDs enabled
                setLimelightLeds(true);
            }
        }
    }

    public void setRobotLEDs() {
        if (isLimelightTargetFound()) {
            blinken.set(RevBlinken.ColorPattern.GREEN); // Green
        } else {
            blinken.set(RevBlinken.ColorPattern.FIRE_LARGE); // Flame
        }
    }

    public void forceLimelightLedsOn(boolean state) {
        forceLimelightLEDOn = state;
    }
}
