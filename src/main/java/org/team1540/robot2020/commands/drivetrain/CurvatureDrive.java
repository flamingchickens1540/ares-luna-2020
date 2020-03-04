package org.team1540.robot2020.commands.drivetrain;

import edu.wpi.first.wpilibj.controller.SimpleMotorFeedforward;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpiutil.math.MathUtil;
import org.team1540.robot2020.RamseteConfig;
import org.team1540.robot2020.utils.ChickenXboxController.Axis;

import java.util.function.BooleanSupplier;

public class CurvatureDrive extends CommandBase {
    public static final double QUICK_STOP_THRESH = 0.2;
    public static final double QUICK_STOP_ALPHA = 0.1;

    private final DriveTrain driveTrain;
    private final Axis throttle;
    private final Axis turn;
    private final BooleanSupplier quickturn;

    private SimpleMotorFeedforward feedforward = new SimpleMotorFeedforward(RamseteConfig.ksVolts, RamseteConfig.kvVoltSecondsPerMeter);

    private double quickStopAccum;

    public CurvatureDrive(DriveTrain driveTrain, Axis throttle, Axis turn, BooleanSupplier quickturn) {
        this.driveTrain = driveTrain;
        this.throttle = throttle;
        this.turn = turn;
        this.quickturn = quickturn;

        addRequirements(driveTrain);
    }

    @Override
    public void execute() {
        double xSpeed = throttle.withDeadzone(0.1).value();
        double zRotation = turn.withDeadzone(0.1).value();

        double angularPower;
        boolean overPower;

        if (quickturn.getAsBoolean()) {
            if (Math.abs(xSpeed) < QUICK_STOP_THRESH) {
                quickStopAccum = (1 - QUICK_STOP_ALPHA) * quickStopAccum
                        + QUICK_STOP_ALPHA * MathUtil.clamp(zRotation, -1.0, 1.0) * 2;
            }
            overPower = true;
            angularPower = zRotation;
        } else {
            overPower = false;
            angularPower = Math.abs(xSpeed) * zRotation - quickStopAccum;

            if (quickStopAccum > 1) {
                quickStopAccum -= 1;
            } else if (quickStopAccum < -1) {
                quickStopAccum += 1;
            } else {
                quickStopAccum = 0.0;
            }
        }

        double leftMotorOutput = xSpeed + angularPower;
        double rightMotorOutput = xSpeed - angularPower;

        // If rotation is overpowered, reduce both outputs to within acceptable range
        if (overPower) {
            if (leftMotorOutput > 1.0) {
                rightMotorOutput -= leftMotorOutput - 1.0;
                leftMotorOutput = 1.0;
            } else if (rightMotorOutput > 1.0) {
                leftMotorOutput -= rightMotorOutput - 1.0;
                rightMotorOutput = 1.0;
            } else if (leftMotorOutput < -1.0) {
                rightMotorOutput -= leftMotorOutput + 1.0;
                leftMotorOutput = -1.0;
            } else if (rightMotorOutput < -1.0) {
                leftMotorOutput -= rightMotorOutput + 1.0;
                rightMotorOutput = -1.0;
            }
        }

        // Normalize the wheel speeds
        double maxMagnitude = Math.max(Math.abs(leftMotorOutput), Math.abs(rightMotorOutput));
        if (maxMagnitude > 1.0) {
            leftMotorOutput /= maxMagnitude;
            rightMotorOutput /= maxMagnitude;
        }


        driveTrain.setVoltage(
                feedforward.calculate(leftMotorOutput * DriveTrain.driveTrueMaxSpeedMetersPerSecond),
                feedforward.calculate(rightMotorOutput * DriveTrain.driveTrueMaxSpeedMetersPerSecond)
        );
    }
}
