package org.team1540.robot2020.commands.drivetrain;

import edu.wpi.first.wpilibj.Notifier;
import edu.wpi.first.wpilibj.SlewRateLimiter;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import org.team1540.robot2020.LocalizationManager;
import org.team1540.robot2020.utils.ChickenXboxController;
import org.team1540.robot2020.utils.ModifiedMiniPID;
import org.team1540.robot2020.utils.PIDConfig;

import java.util.function.Supplier;

import static org.team1540.robot2020.utils.ChickenXboxController.XboxAxis.*;

public class PointToError extends CommandBase {


    private final LocalizationManager localizationManager;
    private final ChickenXboxController.Axis throttleAxis;
    private DriveTrain driveTrain;
    private Supplier<Double> errorSupplier;
    private ChickenXboxController driver;

    private ModifiedMiniPID pointController = new ModifiedMiniPID(0, 0, 0);

    private PIDConfig config;
    private boolean testingMode;
    private boolean useThrottle;

    private Notifier notifier = new Notifier(this::run);


    public PointToError(DriveTrain driveTrain, LocalizationManager localizationManager, Supplier<Double> errorSupplier, ChickenXboxController driver, boolean testingMode, boolean useThrottle) {
        this.localizationManager = localizationManager;
        this.driveTrain = driveTrain;
        this.errorSupplier = errorSupplier;
        this.driver = driver;
        this.throttleAxis = driver.getAxis(LEFT_X);
        this.testingMode = testingMode;
        this.useThrottle = useThrottle;

        setPID(new PIDConfig(0.4, 0, 2, 0, 1, 0.055));
        addRequirements(driveTrain);

        SmartDashboard.putNumber("distanceCalibration/initialDistanceMeters", 0);
    }

    @Override
    public void initialize() {
        localizationManager.ignoreLimelight(false);
        localizationManager.forceLimelightLedsOn(true);
        double p = SmartDashboard.getNumber("pointToTarget/P", 0);
        double i = SmartDashboard.getNumber("pointToTarget/I", 0);
        double d = SmartDashboard.getNumber("pointToTarget/D", 0);
        double IMax = SmartDashboard.getNumber("pointToTarget/IMax", 0);
        double outputMax = SmartDashboard.getNumber("pointToTarget/outputMax", 0);
        double C = SmartDashboard.getNumber("pointToTarget/C", 0);
        config = new PIDConfig(p, i, d, IMax, outputMax, C);
        setPID(config);
        notifier.startPeriodic(0.01);
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
    }

    private void run() {
        double leftMotors = 0;
        double rightMotors = 0;

        if (testingMode) { // TODO: Don't do this
            leftMotors += driver.getAxis(LEFT_X).withDeadzone(0.2).value();
            rightMotors += driver.getAxis(RIGHT_X).withDeadzone(0.2).value();
            double triggerValues = driver.getAxis(LEFT_TRIG).withDeadzone(0.2).value() - driver.getAxis(RIGHT_TRIG).withDeadzone(0.2).value();
            leftMotors += triggerValues;
            rightMotors -= triggerValues;
        } else {
            double throttle = useThrottle ? driveTrain.getThrottleRateLimiter().calculate(throttleAxis.withDeadzone(0.15).value()) : 0;
            leftMotors += throttle;
            rightMotors += throttle;
            double actualCValue = (Math.abs(throttle) > 0.2 || localizationManager.hasReachedPointGoal()) ? 0 : config.c;
            SmartDashboard.putNumber("pointToTarget/actualCValue", actualCValue);
            pointController.setC(actualCValue);
        }

        double pidOutput = pointController.getOutput(errorSupplier.get());

        SmartDashboard.putNumber("pointToTarget/PIDOutput", pidOutput);

        driveTrain.setPercent(leftMotors + pidOutput, rightMotors - pidOutput);
        SmartDashboard.putNumber("pointToTarget/currentAngle", Math.toDegrees(localizationManager.getYawRadians()));
        SmartDashboard.putBoolean("pointToTarget/hasReachedShooterGoal", localizationManager.hasReachedPointGoal());
    }

    @Override
    public void end(boolean interrupted) {
        notifier.stop();
        localizationManager.forceLimelightLedsOn(false);
    }
}
