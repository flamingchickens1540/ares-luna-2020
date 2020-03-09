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
import edu.wpi.first.wpiutil.math.MathUtil;
import org.apache.commons.math3.geometry.euclidean.threed.Vector3D;
import org.apache.commons.math3.geometry.euclidean.twod.Vector2D;
import org.jetbrains.annotations.NotNull;
import org.team1540.robot2020.commands.drivetrain.DriveTrain;
import org.team1540.robot2020.commands.hood.Hood;
import org.team1540.robot2020.commands.shooter.Shooter;
import org.team1540.robot2020.utils.Timer;
import org.team1540.robot2020.utils.*;
import org.team1540.rooster.datastructures.threed.Transform3D;
import org.team1540.rooster.datastructures.utils.RotationUtils;
import org.team1540.rooster.wrappers.RevBlinken;

import javax.annotation.Nullable;
import java.util.function.Consumer;

import static edu.wpi.first.wpilibj2.command.CommandGroupBase.sequence;


public class LocalizationManager extends CommandBase {

    private Runnable onNavxZero;
    private double[] DISTANCE = new double[]{1.901766083, 2.310501969, 3.137942909, 4.573503095, 6.597244189, 7.3940, 7.394777626, 8.012866039, 8.780491971, 10.71451055, 10.715, 11.67940127};
    private double[] HOOD = new double[]{-23.56041718, -19.48951874, -17.72308197, -13.48553925, -11.13584442, -11.26201859, -7.807411194, -7.245451355, -6.316789246, -5.995329285, -9.38, -9.38};
    private double[] FLYWHEEL = new double[]{1643.784083, 1882.776796, 1988.624347, 2498.950448, 2982.554941, 3014.254268, 4629.654212, 4608.690124, 5493.781097, 5643.453423, 5736.983746, 5736.983746};

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
    private final Timer buttonClickTimer = new Timer();
    private Limelight limelight = new Limelight("limelight");
    private LIDARLite lidar = new LIDARLite(I2C.Port.kOnboard);
    private NavX navx = new NavX(SPI.Port.kMXP);
    private DriveTrain driveTrain;
    private Shooter shooter;
    private Hood hood;
    private RevBlinken blinken = new RevBlinken(7);

    private DigitalOutput buttonLed = new DigitalOutput(4);
    private DigitalInput buttonSignal = new DigitalInput(5);

    private boolean limelightBlinkState = false;

    @Nullable
    private Transform3D odomToHexagon = null;
    private boolean acceptLimelight = true;

    public LocalizationManager(DriveTrain driveTrain, Shooter shooter, Hood hood, Consumer<Boolean> zeroHoodFunction) {
        this.driveTrain = driveTrain;
        this.shooter = shooter;
        this.hood = hood;

        // TODO: This is currently being done for the odometry-- remove this in future
        driveTrain.resetEncoders();
        navx.zeroYaw();

        odometry = new DifferentialDriveOdometry(new Rotation2d(0));

        buttonClickTimer.start();

        var button = new Trigger(buttonSignal::get);

        Notifier buttonBlinkNotifier = new Notifier(() -> buttonLed.set(!buttonLed.get()));
        buttonBlinkNotifier.startPeriodic(0.4);

        Notifier limelightBlinkOnNotifier = new Notifier(() -> {
        });
        limelightBlinkOnNotifier.setHandler(() -> {
            limelightBlinkState = !limelightBlinkState;
            limelightBlinkOnNotifier.startSingle(limelightBlinkState ? 0.1 : 2);
        });
        limelightBlinkOnNotifier.startSingle(0.00001);

        CommandBase zeroNavxCommand = sequence(
                new InstCommand(() -> {
                    buttonBlinkNotifier.stop();
                    limelightBlinkOnNotifier.stop();
                    buttonBlinkNotifier.startPeriodic(0.07);
                    limelightBlinkState = true;
                }, true),
                new WaitCommand(1),
                new InstCommand(() -> {
                    limelightBlinkState = false;
                    navx.zeroYaw();
                    System.out.println("NavX Zeroed!" + navx.getYawRadians());
                    updateOdometry();
                    odomToHexagon = getOdomToRobot().add(new Transform3D(1.0934992, 0, HEXAGON_HEIGHT, 0, 0, 0));
                    buttonBlinkNotifier.stop();
                    buttonLed.set(true);
                    if (onNavxZero != null) onNavxZero.run();
                }, true)
        );
        button.whenActive(zeroNavxCommand);
        button.whenActive(() -> {
            if (buttonClickTimer.get() <= 1) {
                zeroHoodFunction.accept(true);
            }
            buttonClickTimer.reset();
        });
        SmartDashboard.putData("localizationManager/zeroYaw", zeroNavxCommand);
    }

    public synchronized void setOnNavxZeroCallback(Runnable onNavxZero) {
        this.onNavxZero = onNavxZero;
    }

    public void initialize() {
        lidar.startMeasuring();
    }

    private synchronized void updateOdomToHexagonTransform() {
        SmartDashboard.putBoolean("localizationManager/isAcceptingLimelight", acceptLimelight);
        boolean lookingAtCorrectTarget = Math.abs(getYawRadians()) < Math.PI / 2;
        SmartDashboard.putBoolean("localizationManager/isLookingAtCorrectTarget", lookingAtCorrectTarget);
        if (acceptLimelight && isLimelightTargetFound() && lookingAtCorrectTarget) {
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

    public synchronized void ignoreLimelight(boolean state) {
        acceptLimelight = !state;
    }

    @NotNull
    private synchronized Transform3D getOdomToRobot() {
        return new Transform3D(odometry.getPoseMeters().getTranslation().getX(), odometry.getPoseMeters().getTranslation().getY(), odometry.getPoseMeters().getRotation().getRadians());
    }

    public synchronized boolean useLidarForDistanceEst() {
        if (!isLimelightTargetFound()) return false;
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
        SmartDashboard.putBoolean("ShooterLineUpSequence/isLimelightTargetFound", limelight.isTargetFound());
        SmartDashboard.putNumber("ShooterLineUpSequence/getDistanceToSelectedTarget", getDistanceToSelectedTarget());
        SmartDashboard.putBoolean("ShooterLineUpSequence/isReadyToShootAll", isLinedUp());
        SmartDashboard.putBoolean("ShooterLineUpSequence/isHoodGoal", hasReachedHoodGoal());
        SmartDashboard.putBoolean("ShooterLineUpSequence/isShooterGoal", hasReachedShooterGoal());
        SmartDashboard.putBoolean("ShooterLineUpSequence/isPointGoal", hasReachedPointGoal());


        if (odomToHexagon != null) {
            putTransform(odomToHexagon, "localizationManager/odomToHexagon");
            putTransform(getRobotToRearHoleTransform(), "localizationManager/robotToRearHole");
            putTransform(getRobotToHexagonTransform(), "localizationManager/robotToHexagon");
        }

        Pose2d poseMeters = odometry.getPoseMeters();
        Translation2d poseTranslation = poseMeters.getTranslation();
        SmartDashboard.putNumber("localizationManager/odometry/X", poseTranslation.getX());
        SmartDashboard.putNumber("localizationManager/odometry/Y", poseTranslation.getY());
        SmartDashboard.putNumber("localizationManager/odometry/rotationDegrees", poseMeters.getRotation().getDegrees());

        limelight.setLeds(getLimelightLEDState());
        setRobotLEDs();
    }

    public synchronized void putTransform(Transform3D transform3D, String prefix) {
        if (transform3D == null) return;
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

    private synchronized void updateOdometry() {
        odometry.update(new Rotation2d(navx.getYawRadians()), driveTrain.getDistanceLeft(), driveTrain.getDistanceRight());
    }

    @Override
    public boolean runsWhenDisabled() {
        return true;
    }

    public synchronized void resetOdometry(Pose2d pose) {
        odometry.resetPosition(pose, new Rotation2d(navx.getYawRadians()));
    }

    public synchronized Pose2d odometryGetPose() {
        return odometry.getPoseMeters();
    }

    @Nullable
    public synchronized Transform3D getRobotToHexagonTransform() {
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

    public synchronized boolean getLimelightLEDState() {
        if (RobotState.isDisabled()) return limelightBlinkState;
        if (forceLimelightLEDOn || getRobotToHexagonTransform() == null) return true;
        double x = getRobotToHexagonTransform().getPosition().getX();
        double y = getRobotToHexagonTransform().getPosition().getY();
        return Math.abs(Math.atan2(y, x)) < Math.toRadians(40);
    }

    public void setRobotLEDs() {
        if (isLimelightTargetFound()) {
            blinken.set(RevBlinken.ColorPattern.GREEN); // Green
        } else {
            blinken.set(RevBlinken.ColorPattern.FIRE_LARGE); // Flame
        }
    }

    public synchronized void forceLimelightLedsOn(boolean state) {
        forceLimelightLEDOn = state;
    }

    public synchronized double getDistanceToSelectedTarget() {
        Transform3D robotToRearHoleTransform = getSelectedTarget();
        if (robotToRearHoleTransform == null) return 4; // default m
        Vector3D position = robotToRearHoleTransform.getPosition();
        Vector2D pos2d = new Vector2D(position.getX(), position.getY());
        return pos2d.getNorm() + (selectedInnerPort ? 0 : 0.5);
    }

    private boolean selectedInnerPort = false;

    public synchronized Transform3D getSelectedTarget() {
        return selectedInnerPort ? getRobotToRearHoleTransform() : getRobotToHexagonTransform();
    }

    public synchronized void selectTarget() {
        selectedInnerPort = shouldTargetInnerPort();
    }

    public double getShooterRPMForSelectedGoal() {
        double norm = MathUtil.clamp(getDistanceToSelectedTarget(), DISTANCE[0] + 0.000000001, Double.MAX_VALUE);
        return MathUtil.clamp(LookupTableUtils.getDoubleLookupTable(norm, DISTANCE, FLYWHEEL), 500, 5800);
    }

    public double getHoodTicksForSelectedGoal() {
        double norm = MathUtil.clamp(getDistanceToSelectedTarget(), DISTANCE[0] + 0.000000001, Double.MAX_VALUE);
        return MathUtil.clamp(LookupTableUtils.getDoubleLookupTable(norm, DISTANCE, HOOD), -292, -1) - Hood.offset;
    }

    public double getPointErrorForSelectedGoal() {
        Transform3D robotToGoalTransform = getSelectedTarget();
        if (robotToGoalTransform == null) return 0;
        Vector3D targetPosition = robotToGoalTransform.getPosition();
        return Math.atan2(targetPosition.getY(), targetPosition.getX());
    }

    public boolean hasReachedPointGoal() {
        return Math.abs(Math.toDegrees(getRate())) < 20 && Math.abs(getPointErrorForSelectedGoal()) < MathUtil.clamp(Math.atan(0.06 / getDistanceToSelectedTarget()), Math.toRadians(0.5), Math.toRadians(3));
    }

    public boolean hasReachedShooterGoal() {
        return Math.abs(shooter.getVelocityRPM() - getShooterRPMForSelectedGoal()) < 100;
    }

    public boolean hasReachedHoodGoal() {
        return Math.abs(getHoodTicksForSelectedGoal() - hood.getPosition()) < 0.1;
    }

    public boolean isLinedUp() {
        return hasReachedPointGoal() && hasReachedShooterGoal() && hasReachedHoodGoal() && (isLimelightTargetFound() || getDistanceToSelectedTarget() < 2.3);
    }
}
