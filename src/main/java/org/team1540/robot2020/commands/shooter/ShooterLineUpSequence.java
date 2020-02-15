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
