package org.team1540.robot2020.commands.drivetrain;

import static edu.wpi.first.networktables.EntryListenerFlags.kUpdate;

import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.GenericHID.Hand;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import org.apache.commons.math3.geometry.euclidean.twod.Vector2D;
import org.team1540.robot2020.shouldbeinrooster.ConstantOptimizer;
import org.team1540.robot2020.shouldbeinrooster.ModifiedMiniPID;
import org.team1540.robot2020.shouldbeinrooster.NavX;
import org.team1540.robot2020.subsystems.DriveTrain;
import org.team1540.rooster.util.ChickenXboxController;
import org.team1540.rooster.util.ChickenXboxController.XboxAxis;
import org.team1540.rooster.util.TrigUtils;
import org.team1540.rooster.wrappers.Limelight;

public class PointToTarget extends CommandBase {


    private final boolean useShuffleboardPID = true;
    private final int finishedSamples = 50;
    private double timeout = 3;
    private double goalAngleToleranceDegrees = 2;
    private double lastTargetAngle = 0;

    private final NavX navx;
    private final Limelight limelight;
    private DriveTrain driveTrain;
    private ChickenXboxController driver;

    private ModifiedMiniPID pointController = new ModifiedMiniPID(0, 0, 0);
    private Timer timeoutTimer = new Timer();

    private double sampleCount;
    private double errorAccumulator;

    private ConstantOptimizer optimizer = new ConstantOptimizer(0, true, 0.1);
    private PIDConfig config;

    public PointToTarget(NavX navx, DriveTrain driveTrain, ChickenXboxController driver, Limelight limelight, PIDConfig config) {
        this.navx = navx;
        this.driveTrain = driveTrain;
        this.driver = driver;
        this.limelight = limelight;

        setPID(config);
        addRequirements(driveTrain);

        if (useShuffleboardPID) {
            NetworkTableInstance.getDefault().getTable("SmartDashboard").getSubTable("Shooter").addEntryListener((table, key, entry, value, flags) -> {
                double p = SmartDashboard.getNumber("Shooter/P", 0);
                double i = SmartDashboard.getNumber("Shooter/I", 0);
                double d = SmartDashboard.getNumber("Shooter/D", 0);
                double IMax = SmartDashboard.getNumber("Shooter/IMax", 0);
                double outputMax = SmartDashboard.getNumber("Shooter/outputMax", 0);
                double C = SmartDashboard.getNumber("Shooter/C", 0);
                setPID(new PIDConfig(p, i, d, IMax, outputMax, C));
            }, kUpdate);
        }
    }

    @Override
    public void initialize() {
        sampleCount = 0;
        errorAccumulator = 0;
        timeoutTimer.reset();
        timeoutTimer.start();
//        lastTargetAngle = navx.getAngleRadians() + Math.toRadians(90);
    }

    private void setPID(PIDConfig config) {
        this.config = config;
        pointController.setPID(config.p, config.i, config.d);
        pointController.setOutputLimits(config.outputMax);
        pointController.setMaxIOutput(config.iMax);
        pointController.setC(config.c);
        SmartDashboard.putNumber("Shooter/P", config.p);
        SmartDashboard.putNumber("Shooter/I", config.i);
        SmartDashboard.putNumber("Shooter/D", config.d);
        SmartDashboard.putNumber("Shooter/IMax", config.iMax);
        SmartDashboard.putNumber("Shooter/outputMax", config.outputMax);
        SmartDashboard.putNumber("Shooter/C", config.c);
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
        SmartDashboard.putNumber("Shooter/PIDOutput", pidOutput);

        driveTrain.tankDrivePercent(leftMotors, rightMotors);
    }

    private double calculateError() {
        if (limelight.isTargetFound()) {
            Vector2D targetAngles = limelight.getTargetAngles();
            lastTargetAngle = navx.getAngleRadians() - targetAngles.getX();
            SmartDashboard.putNumber("Shooter/limelightTarget", targetAngles.getX());
        }
        SmartDashboard.putNumber("Shooter/currentAngle", navx.getYawRadians());
        SmartDashboard.putNumber("Shooter/targetAngles", lastTargetAngle);
        SmartDashboard.putNumber("Shooter/errorAccumulator", errorAccumulator);
        SmartDashboard.putNumber("Shooter/sampleCount", sampleCount);

        double error = TrigUtils.signedAngleError(lastTargetAngle, navx.getAngleRadians());
        SmartDashboard.putNumber("Shooter/error", Math.toDegrees(error));

        sampleCount++;
        errorAccumulator += Math.abs(error);

        if (sampleCount == finishedSamples) {
            if (hasTimedOut() || hasReachedGoal()) {
                double newP = optimizer.computeNextGuess(hasTimedOut() ? Math.toDegrees(error) + 3 : timeoutTimer.get());
                config.d = newP;
//                setPID(config);
                initialize();
            } else {
                sampleCount = 0;
                errorAccumulator = 0;
            }
        }

        return error;
    }

    @Override
    public boolean isFinished() {
        return false;
    }

    private boolean hasTimedOut() {
        return timeoutTimer.hasPeriodPassed(timeout);
    }

    private boolean hasReachedGoal() {
        return sampleCount == finishedSamples && errorAccumulator < Math.toRadians(goalAngleToleranceDegrees * finishedSamples);
    }
}
