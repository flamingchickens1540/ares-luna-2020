package org.team1540.robot2020.commands.drivetrain;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import org.team1540.robot2020.subsystems.DriveTrain;
import org.team1540.robot2020.utils.ChickenXboxController;
import org.team1540.robot2020.utils.ControlUtils;
import org.team1540.robot2020.utils.MiniPID;
import org.team1540.robot2020.utils.NavX;
import org.team1540.rooster.util.TrigUtils;

import static org.team1540.robot2020.utils.ChickenXboxController.Axis2D;
import static org.team1540.robot2020.utils.ChickenXboxController.Hand.RIGHT;
import static org.team1540.robot2020.utils.ChickenXboxController.XboxAxis.LEFT_X;
import static org.team1540.robot2020.utils.ChickenXboxController.XboxButton.Y;

public class PointDrive extends CommandBase {
    private DriveTrain driveTrain;
    private NavX navx;
    private ChickenXboxController driver;

    private double goalAngle;
    private MiniPID pointController;
    private double angleOffset;
    private double max;
    private double min;
    private double deadzone;

    // TODO: this is pasted from last year's code, feel free to clean up the tuning constant stuff
    // TODO can we pass in the Axis objects and Button objects instead of the entire damn controller
    public PointDrive(DriveTrain driveTrain, NavX navx, ChickenXboxController driver) {
        this.driveTrain = driveTrain;
        this.navx = navx;
        this.driver = driver;
        addRequirements(driveTrain);

        // TODO: hmm, we really need that YAML tuning lib
        SmartDashboard.putNumber("pointDrive/P", 0.5);
        SmartDashboard.putNumber("pointDrive/I", 0);
        SmartDashboard.putNumber("pointDrive/D", 0);
        SmartDashboard.putNumber("pointDrive/max", 0.5);
        SmartDashboard.putNumber("pointDrive/min", 0);
        SmartDashboard.putNumber("pointDrive/deadzone", 0.02);

        pointController = new MiniPID(0, 0, 0);
        driver.getButton(Y).whenPressed(this::zeroAngle);
    }

    @Override
    public void initialize() {
        double p = SmartDashboard.getNumber("pointDrive/P", 0);
        double i = SmartDashboard.getNumber("pointDrive/I", 0);
        double d = SmartDashboard.getNumber("pointDrive/D", 0);
        max = SmartDashboard.getNumber("pointDrive/max", 0);
        min = SmartDashboard.getNumber("pointDrive/min", 0);
        deadzone = SmartDashboard.getNumber("pointDrive/deadzone", 0);
        pointController.setPID(p, i, d);
        setGoalToCurrentAngle();
    }

    public void zeroAngle() {
        angleOffset = navx.getYawRadians();
        setGoalToCurrentAngle();
    }

    public void setGoalToCurrentAngle() {
        goalAngle = navx.getYawRadians() - angleOffset;
    }

    @Override
    public void execute() {
        Axis2D pointAxis = driver.getAxis2D(RIGHT);
        if (pointAxis.magnitude().value() > 0.5) goalAngle = pointAxis.angle().value();

        double error = TrigUtils.signedAngleError(goalAngle + angleOffset, navx.getYawRadians());
        SmartDashboard.putNumber("pointDrive/error", error);

        double rawPIDOutput = pointController.getOutput(error);
        double angleOutput = ControlUtils.allVelocityConstraints(rawPIDOutput, max, min, deadzone);

        double throttle = driver.getAxis(LEFT_X).withDeadzone(.1).value();

        double leftMotors = throttle + angleOutput;
        double rightMotors = throttle - angleOutput;
        driveTrain.setPercent(leftMotors, rightMotors);
    }
}
