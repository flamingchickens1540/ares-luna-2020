package org.team1540.robot2020.commands.drivetrain;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import org.apache.commons.math3.geometry.euclidean.twod.Vector2D;
import org.team1540.robot2020.utils.*;
import org.team1540.rooster.util.TrigUtils;

import java.util.List;

import static org.team1540.robot2020.utils.ChickenXboxController.XboxAxis.*;

public class PointToTarget extends CommandBase {


    private final NavX navx;
    private final Limelight limelight;
    private final ChickenXboxController.Axis throttleAxis;
    private boolean testingMode;
    private double lastTargetAngle = 0;
    private DriveTrain driveTrain;
    private ChickenXboxController driver;

    private ModifiedMiniPID pointController = new ModifiedMiniPID(0, 0, 0);

    private double finishedDegreesPerSecond = 0.5;
    private double finishedDegrees = 1;
    private double lastError = Double.NEGATIVE_INFINITY;
    private PIDConfig config;

    private boolean foundTarget;


    public PointToTarget(DriveTrain driveTrain, NavX navx, Limelight limelight, ChickenXboxController driver, boolean testingMode) {
        this.navx = navx;
        this.driveTrain = driveTrain;
        this.driver = driver;
        this.limelight = limelight;
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
        lastTargetAngle = navx.getAngleRadians();
        foundTarget = false;

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
            double throttle = throttleAxis.withDeadzone(0.12).value();
            leftMotors += throttle;
            rightMotors += throttle;
            double actualCValue = Math.abs(throttle) > 0.12 ? 0 : config.c;
            SmartDashboard.putNumber("pointToTarget/actualCValue", actualCValue);
            pointController.setC(actualCValue);
        }

        double pidOutput = pointController.getOutput(calculateError());

        if (!foundTarget) {
            pidOutput = 0;
        }

        SmartDashboard.putNumber("pointToTarget/PIDOutput", pidOutput);

        driveTrain.setPercent(leftMotors + pidOutput, rightMotors - pidOutput);
    }

    private double calculateError() {
        if (limelight.isTargetFound()) {
            foundTarget = true;
            Vector2D targetAngles = limelight.getTargetAngles();
            lastTargetAngle = navx.getAngleRadians() - targetAngles.getX();
            SmartDashboard.putNumber("pointToTarget/limelightTarget", targetAngles.getX());
        }

        double error = TrigUtils.signedAngleError(lastTargetAngle, navx.getAngleRadians());

        lastError = error;

        SmartDashboard.putNumber("pointToTarget/currentAngle", Math.toDegrees(navx.getYawRadians()));
        SmartDashboard.putNumber("pointToTarget/error", Math.toDegrees(error));
        SmartDashboard.putBoolean("pointToTarget/hasReachedGoal", hasReachedGoal());

        return error;
    }

    public boolean hasReachedGoal() {
        return Math.abs(navx.getRate()) < finishedDegreesPerSecond && Math.abs(Math.toDegrees(lastError)) < finishedDegrees;
    }

    private String coefsToLatexString(List<Double> coefs) {
        String latexString = "";
        for (int i = 0; i < coefs.size(); i++) {
            latexString += coefs.get(i) + "x^{" + i + "}+";
        }
        return latexString;
    }
}
