package org.team1540.robot2020.commands.drivetrain;

import static edu.wpi.first.networktables.EntryListenerFlags.kUpdate;

import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.GenericHID.Hand;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import org.apache.commons.math3.geometry.euclidean.twod.Vector2D;
import org.team1540.robot2020.shouldbeinrooster.NavX;
import org.team1540.robot2020.subsystems.DriveTrain;
import org.team1540.rooster.util.ChickenXboxController;
import org.team1540.rooster.util.MiniPID;
import org.team1540.rooster.util.TrigUtils;
import org.team1540.rooster.wrappers.Limelight;

public class PointToTarget extends CommandBase {


    private final boolean useShuffleboardPID = true;
    private double timeout = 3;
    private double goalAngleToleranceDegrees = 10;
    private double lastTargetAngle = 0;

    private final NavX navx;
    private final Limelight limelight;
    private DriveTrain driveTrain;
    private ChickenXboxController driver;

    private MiniPID pointController = new MiniPID(0, 0, 0);
    private Timer timeoutTimer = new Timer();

    private int sampleCount;
    private int errorAccumulator;

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
                setPID(new PIDConfig(p, i, d, IMax));
            }, kUpdate);
        }
    }

    @Override
    public void initialize() {
        sampleCount = 0;
        errorAccumulator = 0;
        timeoutTimer.reset();
    }

    private void setPID(PIDConfig config) {
        pointController.setPID(config.p, config.i, config.d);
        pointController.setOutputLimits(0.3);
        pointController.setMaxIOutput(config.IMax);
        SmartDashboard.putNumber("Shooter/P", config.p);
        SmartDashboard.putNumber("Shooter/I", config.i);
        SmartDashboard.putNumber("Shooter/D", config.d);
        SmartDashboard.putNumber("Shooter/IMax", config.IMax);
    }

    @Override
    public void execute() {
        double leftMotors = driver.getRectifiedXAxis(Hand.kLeft).withDeadzone(0.2).value();
        double rightMotors = driver.getRectifiedXAxis(Hand.kRight).withDeadzone(0.2).value();

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
            SmartDashboard.putNumber("Shooter/currentAngle", navx.getAngleRadians());
            SmartDashboard.putNumber("Shooter/targetAngles", lastTargetAngle);
        }
        double error = TrigUtils.signedAngleError(lastTargetAngle, navx.getAngleRadians());
        if (errorAccumulator > Math.toRadians(goalAngleToleranceDegrees)) {
            sampleCount = 0;
            errorAccumulator = 0;
        } else {
            sampleCount++;
            errorAccumulator += Math.abs(error);
        }
        return error;
    }

    @Override
    public boolean isFinished() {
        if (useShuffleboardPID) {
            return false;
        }
        return hasTimedOut() || hasReachedGoal();
    }

    private boolean hasTimedOut() {
        return timeoutTimer.hasPeriodPassed(timeout);
    }

    private boolean hasReachedGoal() {
        return sampleCount > 10 || errorAccumulator < Math.toRadians(goalAngleToleranceDegrees);
    }
}
