package org.team1540.robot2020.commands.drivetrain;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import org.apache.commons.math3.geometry.euclidean.threed.Vector3D;
import org.team1540.robot2020.LocalizationManager;
import org.team1540.robot2020.utils.ChickenXboxController;
import org.team1540.robot2020.utils.ModifiedMiniPID;
import org.team1540.robot2020.utils.PIDConfig;
import org.team1540.rooster.datastructures.threed.Transform3D;

import java.util.function.Supplier;

import static org.team1540.robot2020.utils.ChickenXboxController.XboxAxis.*;

public class PointToTransform extends CommandBase {


    private final LocalizationManager localizationManager;
    private final ChickenXboxController.Axis throttleAxis;
    private DriveTrain driveTrain;
    private Supplier<Transform3D> targetSupplier;
    private ChickenXboxController driver;

    private ModifiedMiniPID pointController = new ModifiedMiniPID(0, 0, 0);

    private double finishedDegreesPerSecond = 0.5;
    private double finishedDegrees = 0.3;
    private double lastError = Double.NEGATIVE_INFINITY;
    private PIDConfig config;
    private boolean testingMode;


    public PointToTransform(DriveTrain driveTrain, LocalizationManager localizationManager, Supplier<Transform3D> targetSupplier, ChickenXboxController driver, boolean testingMode) {
        this.localizationManager = localizationManager;
        this.driveTrain = driveTrain;
        this.targetSupplier = targetSupplier;
        this.driver = driver;
        this.throttleAxis = driver.getAxis(LEFT_X);
        this.testingMode = testingMode;

        setPID(new PIDConfig(0.4, 0.07, 0.9, 0.008, 0.25, 0.013));
        addRequirements(driveTrain);

        SmartDashboard.putNumber("pointToTarget/finishedDegrees", finishedDegrees);
        SmartDashboard.putNumber("pointToTarget/finishedDegreesPerSecond", finishedDegreesPerSecond);
        SmartDashboard.putNumber("distanceCalibration/initialDistanceMeters", 0);
    }

    @Override
    public void initialize() {

        double p = SmartDashboard.getNumber("pointToTarget/P", 0);
        double i = SmartDashboard.getNumber("pointToTarget/I", 0);
        double d = SmartDashboard.getNumber("pointToTarget/D", 0);
        double IMax = SmartDashboard.getNumber("pointToTarget/IMax", 0);
        double outputMax = SmartDashboard.getNumber("pointToTarget/outputMax", 0);
        double C = SmartDashboard.getNumber("pointToTarget/C", 0);
        double finishedDegrees = SmartDashboard.getNumber("pointToTarget/finishedDegrees", 0);
        double finishedDegreesPerSecond = SmartDashboard.getNumber("pointToTarget/finishedDegreesPerSecond", 0);
        this.finishedDegrees = finishedDegrees;
        this.finishedDegreesPerSecond = finishedDegreesPerSecond;
        config = new PIDConfig(p, i, d, IMax, outputMax, C);
        setPID(config);
    }

    private void setPID(PIDConfig config) {
        pointController.setPID(config.p, config.i, config.d);
        pointController.setOutputLimits(config.outputMax);
        pointController.setMaxIOutput(config.iMax);
        pointController.setC(config.c);
        SmartDashboard.putNumber("pointToTarget/P", config.p);
        SmartDashboard.putNumber("pointToTarget/I", config.i);
        SmartDashboard.putNumber("pointToTarget/D", config.d);
        SmartDashboard.putNumber("pointToTarget/IMax", config.iMax);
        SmartDashboard.putNumber("pointToTarget/outputMax", config.outputMax);
        SmartDashboard.putNumber("pointToTarget/C", config.c);
    }

    @Override
    public void execute() {
        double leftMotors = 0;
        double rightMotors = 0;

        if (testingMode) { // TODO: Don't do this
            leftMotors += driver.getAxis(LEFT_X).withDeadzone(0.2).value();
            rightMotors += driver.getAxis(RIGHT_X).withDeadzone(0.2).value();
            double triggerValues = driver.getAxis(LEFT_TRIG).withDeadzone(0.2).value() - driver.getAxis(RIGHT_TRIG).withDeadzone(0.2).value();
            leftMotors += triggerValues;
            rightMotors -= triggerValues;
        } else {
            double throttle = throttleAxis.withDeadzone(0.2).value();
            leftMotors += throttle;
            rightMotors += throttle;
            double actualCValue = Math.abs(throttle) > 0.2 ? 0 : config.c;
            SmartDashboard.putNumber("pointToTarget/actualCValue", actualCValue);
            pointController.setC(actualCValue);
        }

        double pidOutput = pointController.getOutput(calculateError());

        SmartDashboard.putNumber("pointToTarget/PIDOutput", pidOutput);

        driveTrain.setPercent(leftMotors + pidOutput, rightMotors - pidOutput);
    }

    private double calculateError() {
        Transform3D robotToGoalTransform = targetSupplier.get();
        if (robotToGoalTransform == null) return 0;

        Vector3D targetPosition = robotToGoalTransform.getPosition();
        double targetAngle = Math.atan2(targetPosition.getY(), targetPosition.getX());
//                - Math.toRadians(1.5);
        SmartDashboard.putNumber("pointToTarget/targetAngle", targetAngle);

        lastError = targetAngle;

        SmartDashboard.putNumber("pointToTarget/currentAngle", Math.toDegrees(localizationManager.getYawRadians()));
        SmartDashboard.putNumber("pointToTarget/error", Math.toDegrees(targetAngle));
        SmartDashboard.putBoolean("pointToTarget/hasReachedGoal", hasReachedGoal());

        return targetAngle;
    }

    public boolean hasReachedGoal() {
        return Math.abs(localizationManager.getRate()) < finishedDegreesPerSecond && Math.abs(Math.toDegrees(lastError)) < finishedDegrees;
    }
}
