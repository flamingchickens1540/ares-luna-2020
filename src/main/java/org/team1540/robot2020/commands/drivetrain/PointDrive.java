package org.team1540.robot2020.commands.drivetrain;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import org.team1540.robot2020.subsystems.DriveTrain;
import org.team1540.robot2020.utils.ChickenXboxController;
import org.team1540.robot2020.utils.ControlUtils;
import org.team1540.robot2020.utils.InstCommand;
import org.team1540.robot2020.utils.MiniPID;
import org.team1540.rooster.util.TrigUtils;

import static org.team1540.robot2020.utils.ChickenXboxController.Axis2D;
import static org.team1540.robot2020.utils.ChickenXboxController.Hand.RIGHT;
import static org.team1540.robot2020.utils.ChickenXboxController.XboxAxis.LEFT_X;
import static org.team1540.robot2020.utils.ChickenXboxController.XboxButton.Y;

public class PointDrive extends CommandBase {
    private DriveTrain driveTrain;
    private ChickenXboxController driver;

    private double goalAngle;
    private MiniPID pointController;
    private double angleOffset;
    private double max;
    private double min;
    private double deadzone;

    // TODO: this is pasted from last year's code, feel free to clean up the tuning constant stuff
    public PointDrive(DriveTrain driveTrain, ChickenXboxController driver) {
        this.driveTrain = driveTrain;
        this.driver = driver;
        addRequirements(driveTrain);

        // TODO: hmm, we really need that YAML tuning lib
        SmartDashboard.putNumber("PointDrive/P", 0.5);
        SmartDashboard.putNumber("PointDrive/I", 0);
        SmartDashboard.putNumber("PointDrive/D", 0);
        SmartDashboard.putNumber("PointDrive/max", 0.5);
        SmartDashboard.putNumber("PointDrive/min", 0);
        SmartDashboard.putNumber("PointDrive/deadzone", 0.02);

        pointController = new MiniPID(0, 0, 0);
        driver.getButton(Y).whenPressed(new InstCommand(this::zeroAngle));
    }

    @Override
    public void initialize() {
        double p = SmartDashboard.getNumber("PointDrive/P", 0);
        double i = SmartDashboard.getNumber("PointDrive/I", 0);
        double d = SmartDashboard.getNumber("PointDrive/D", 0);
        max = SmartDashboard.getNumber("PointDrive/max", 0);
        min = SmartDashboard.getNumber("PointDrive/min", 0);
        deadzone = SmartDashboard.getNumber("PointDrive/deadzone", 0);
        pointController.setPID(p, i, d);
        setGoalToCurrentAngle();
    }

    public void zeroAngle() {
        angleOffset = driveTrain.navx.getYawRadians();
        setGoalToCurrentAngle();
    }

    public void setGoalToCurrentAngle() {
        goalAngle = driveTrain.navx.getYawRadians() - angleOffset;
    }

    @Override
    public void execute() {
        Axis2D pointAxis = driver.getAxis2D(RIGHT);
        if (pointAxis.magnitude().value() > 0.5) goalAngle = pointAxis.angle().value();

        double error = TrigUtils.signedAngleError(goalAngle + angleOffset, driveTrain.navx.getYawRadians());
        SmartDashboard.putNumber("PointDrive/error", error);

        double rawPIDOutput = pointController.getOutput(error);
        double angleOutput = ControlUtils.allVelocityConstraints(rawPIDOutput, max, min, deadzone);

        double throttle = driver.getAxis(LEFT_X).withDeadzone(.1).value();

        double leftMotors = throttle + angleOutput;
        double rightMotors = throttle - angleOutput;
        driveTrain.tankDrivePercent(leftMotors, rightMotors);
    }
}