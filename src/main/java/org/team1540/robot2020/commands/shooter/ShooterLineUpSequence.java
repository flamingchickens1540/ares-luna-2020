package org.team1540.robot2020.commands.shooter;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import org.team1540.robot2020.commands.drivetrain.DriveTrain;
import org.team1540.robot2020.commands.drivetrain.PointToTarget;
import org.team1540.robot2020.commands.hood.Hood;
import org.team1540.robot2020.commands.hood.HoodSetPosition;
import org.team1540.robot2020.utils.ChickenXboxController;
import org.team1540.robot2020.utils.NavX;
import org.team1540.rooster.wrappers.Limelight;

import java.util.function.BooleanSupplier;

public class ShooterLineUpSequence extends SequentialCommandGroup {
    Shooter shooter;
    PointToTarget pointingCommand;
    ShooterSpinUp shootingCommand;
    HoodSetPosition hoodCommand;

    double[] HOOD = new double[]{-71.78778076171875, -77.28832244873047, -95.57478332519531, -105.16877746582031, -139.78330993652344, -224.01043701171875};
    double[] DISTANCE = new double[]{352.06299212598424, 261.11811023622045, 183.16535433070865, 154.03149606299212, 120.96062992125984, 63.874015748031496};
    double[] FLYWHEEL = new double[]{5000.0, 5000.0, 5000.0, 2870.0, 2480.0, 1780.0};

    public ShooterLineUpSequence(double hoodAngle, double flywheelVelocity, NavX navx, DriveTrain driveTrain, ChickenXboxController driverController, Limelight limelight, Shooter shooter, Hood hood) {
        this.shooter = shooter;
        this.pointingCommand = new PointToTarget(navx, driveTrain, driverController, limelight);
        this.shootingCommand = new ShooterSpinUp(shooter, 5000); //TODO: RPM constant
        this.hoodCommand = new HoodSetPosition(hood, hoodAngle);

        addCommands(parallel(
                pointingCommand,
                shootingCommand,
                hoodCommand
        ));
    }

    private double line(double x1, double y1, double x2, double y2, double x) {
        return ((y2 - y1) / (x2 - x2)) * (x - x1) + y2;
    }

    public double getHoodPosition(double distance) {
        if (distance < HOOD[0]) { // If distance less than smallest recorded distance
            double lowerX = HOOD[0];
            double lowerY = DISTANCE[0];

            double upperX = HOOD[1];
            double upperY = DISTANCE[1];

            return line(lowerX, lowerY, upperX, upperY, distance);
        } else if (distance > HOOD[HOOD.length - 1]) { // If distance greater than largest recorded value
            double lowerX = HOOD[HOOD.length - 2];
            double lowerY = DISTANCE[DISTANCE.length - 2];

            double upperX = HOOD[HOOD.length - 1];
            double upperY = DISTANCE[DISTANCE.length - 1];

            return line(lowerX, lowerY, upperX, upperY, distance);
        } else { // If distance somewhere in the middle
            for (int i = 0; i < HOOD.length - 1; i++) {
                double lowerX = HOOD[i];
                double lowerY = DISTANCE[i];

                double upperX = HOOD[i + 1];
                double upperY = DISTANCE[i + 1];

                if ((lowerX < distance) && (distance < upperX)) { // HOOD[i-1] < distance < HOOD[i+1]
                    return line(lowerX, lowerY, upperX, upperY, distance);
                }
            }
        }
        return 0;
    }

    public BooleanSupplier isLinedUp() {
        return () -> {
            if (!isScheduled()) {
                return false;
            } else {
                return (pointingCommand.isPointingAtGoalAndStopped()) && (Math.abs(shooter.getClosedLoopError()) < 100);
            }
        };
    }

}
