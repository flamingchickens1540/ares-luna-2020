package org.team1540.robot2020.commands.drivetrain;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import org.apache.commons.math3.geometry.euclidean.twod.Vector2D;
import org.team1540.robot2020.utils.ChickenXboxController;
import org.team1540.robot2020.utils.ModifiedMiniPID;
import org.team1540.robot2020.utils.NavX;
import org.team1540.robot2020.utils.PIDConfig;
import org.team1540.rooster.util.TrigUtils;
import org.team1540.rooster.wrappers.Limelight;

import java.util.List;

import static org.team1540.robot2020.utils.ChickenXboxController.XboxAxis.*;

public class PointToTarget extends CommandBase {


    private final NavX navx;
    private final Limelight limelight;
    private double lastTargetAngle = 0;
    private DriveTrain driveTrain;
    private ChickenXboxController driver;

    private ModifiedMiniPID pointController = new ModifiedMiniPID(0, 0, 0);

    private double finishedDegreesPerSecond = 0.1;
    private double finishedDegrees = 0.2;

    public PointToTarget(NavX navx, DriveTrain driveTrain, ChickenXboxController driver, Limelight limelight, PIDConfig config) {
        this.navx = navx;
        this.driveTrain = driveTrain;
        this.driver = driver;
        this.limelight = limelight;

        setPID(config);
        addRequirements(driveTrain);

        SmartDashboard.putNumber("pointToTarget/finishedDegrees", finishedDegrees);
        SmartDashboard.putNumber("pointToTarget/finishedDegreesPerSecond", finishedDegreesPerSecond);
        SmartDashboard.putNumber("distanceCalibration/initialDistanceMeters", 0);

//        NetworkTableInstance.getDefault().getTable("SmartDashboard").getSubTable("pointToTarget").addEntryListener((table, key, entry, value, flags) -> {
//
//        }, kUpdate);
    }

    @Override
    public void initialize() {
        double leftEncoderDistance = driveTrain.getDistanceLeft();
        double rightEncoderDistance = driveTrain.getDistanceRight();
//        calibrationString = "";
//        initialDistanceLeft = leftEncoderDistance;
//        initialDistanceRight = rightEncoderDistance;
        lastTargetAngle = navx.getAngleRadians();
//        actualDistances.clear();
//        measuredDistances.clear();


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
        setPID(new PIDConfig(p, i, d, IMax, outputMax, C));
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
        double leftMotors = driver.getAxis(LEFT_X).withDeadzone(0.2).value();
        double rightMotors = driver.getAxis(RIGHT_X).withDeadzone(0.2).value();
        double triggerValues = driver.getAxis(LEFT_TRIG).withDeadzone(0.2).value() - driver.getAxis(RIGHT_TRIG).withDeadzone(0.2).value();
        leftMotors -= triggerValues;
        rightMotors += triggerValues;

        double pidOutput = pointController.getOutput(calculateError());
//        leftMotors += pidOutput;
//        rightMotors -= pidOutput;

        SmartDashboard.putNumber("pointToTarget/PIDOutput", pidOutput);

        driveTrain.setPercent(leftMotors + pidOutput, rightMotors - pidOutput);
    }

//    private String calibrationString;
//    private double initialDistanceLeft;
//    private double initialDistanceRight;
//    private double counter;

//    private List<Double> actualDistances = new ArrayList<Double>();
//    private List<Double> measuredDistances = new ArrayList<Double>();

    private double calculateError() {
//        double leftEncoderDistance = driveTrain.getDistanceLeft();
//        double rightEncoderDistance = driveTrain.getDistanceRight();
        if (limelight.isTargetFound()) {
            Vector2D targetAngles = limelight.getTargetAngles();
            lastTargetAngle = navx.getAngleRadians() - targetAngles.getX();

//            final double limelightHeight = 49;
//            final double targetHeight = 89.5;

//            double offsetAngle = Math.toRadians(0);
//            double distanceToTarget = UnitsUtils.inchesToMeters(targetHeight - limelightHeight) / Math.tan(targetAngles.getY() + offsetAngle);

//            double initialDistance = UnitsUtils.inchesToMeters(18);
//            double actualDistance = initialDistance + (Math.abs(initialDistanceLeft - leftEncoderDistance) + Math.abs(initialDistanceRight - rightEncoderDistance)) / 2;

//            counter++;
//            if (counter % 10 == 0 && actualDistance > UnitsUtils.inchesToMeters(100)) {
//                calibrationString += actualDistance + "," + distanceToTarget + "n";
//                actualDistances.add(actualDistance);
//                measuredDistances.add(distanceToTarget);
//                SmartDashboard.putString("distanceCalibration/calibrationString", calibrationString);
//            }

//            SmartDashboard.putNumber("distanceCalibration/distanceToTarget", distanceToTarget);
//            SmartDashboard.putNumber("distanceCalibration/actualDistance", actualDistance);
            SmartDashboard.putNumber("pointToTarget/limelightTarget", targetAngles.getX());
        }

        double error = TrigUtils.signedAngleError(lastTargetAngle, navx.getAngleRadians());

        SmartDashboard.putNumber("pointToTarget/currentAngle", Math.toDegrees(navx.getYawRadians()));
        SmartDashboard.putNumber("pointToTarget/error", Math.toDegrees(error));
        SmartDashboard.putBoolean("pointToTarget/reachedGoal", isPointingAtGoalAndStopped(error));

        return error;
    }

    private boolean isPointingAtGoalAndStopped(double error) {
        return Math.abs(navx.getRate()) < finishedDegreesPerSecond && Math.abs(Math.toDegrees(error)) < finishedDegrees;
    }

    @Override
    public boolean isFinished() {
        return false;
    }

    @Override
    public void end(boolean interrupted) {
//        PolyTrendLine t = new PolyTrendLine(1);
//        t.setValues(measuredDistances.toArray(new Double[0]), actualDistances.toArray(new Double[0]));
//        SmartDashboard.putString("distanceCalibration/resultsString", t.getCoef().toString());
//        SmartDashboard.putString("distanceCalibration/desmosString", coefsToLatexString(t.getCoef()));
//        SmartDashboard.putNumberArray("distanceCalibration/results", t.getCoef().toArray(new Double[0]));
    }

    private String coefsToLatexString(List<Double> coefs) {
        String latexString = "";
        for (int i = 0; i < coefs.size(); i++) {
            latexString += coefs.get(i) + "x^{" + i + "}+";
        }
        return latexString;
    }
}
