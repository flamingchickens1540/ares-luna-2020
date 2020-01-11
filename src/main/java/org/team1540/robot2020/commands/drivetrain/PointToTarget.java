package org.team1540.robot2020.commands.drivetrain;

import static edu.wpi.first.networktables.EntryListenerFlags.kUpdate;

import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.GenericHID.Hand;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import org.apache.commons.math3.geometry.euclidean.twod.Vector2D;
import org.team1540.robot2020.shouldbeinrooster.ModifiedMiniPID;
import org.team1540.robot2020.shouldbeinrooster.NavX;
import org.team1540.robot2020.subsystems.DriveTrain;
import org.team1540.rooster.util.ChickenXboxController;
import org.team1540.rooster.util.ChickenXboxController.XboxAxis;
import org.team1540.rooster.util.TrigUtils;
import org.team1540.rooster.wrappers.Limelight;

public class PointToTarget extends CommandBase {


    private double lastTargetAngle = 0;

    private final NavX navx;
    private final Limelight limelight;
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

        NetworkTableInstance.getDefault().getTable("SmartDashboard").getSubTable("pointToTarget").addEntryListener((table, key, entry, value, flags) -> {
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
        }, kUpdate);
    }

    @Override
    public void initialize() {
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
        double leftMotors = driver.getRectifiedXAxis(Hand.kLeft).withDeadzone(0.2).value();
        double rightMotors = driver.getRectifiedXAxis(Hand.kRight).withDeadzone(0.2).value();
        double triggerValues = driver.getAxis(XboxAxis.LEFT_TRIG).withDeadzone(0.2).value() - driver.getAxis(XboxAxis.RIGHT_TRIG).withDeadzone(0.2).value();
        leftMotors += triggerValues;
        rightMotors -= triggerValues;

        double pidOutput = pointController.getOutput(calculateError());
        leftMotors += pidOutput;
        rightMotors -= pidOutput;

        SmartDashboard.putNumber("pointToTarget/PIDOutput", pidOutput);

        driveTrain.tankDrivePercent(leftMotors, rightMotors);
    }

    private double calculateError() {
        if (limelight.isTargetFound()) {
            Vector2D targetAngles = limelight.getTargetAngles();
            lastTargetAngle = navx.getAngleRadians() - targetAngles.getX();
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
}
