package org.team1540.robot2020.commands.shooter;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import org.team1540.robot2020.commands.drivetrain.DriveTrain;
import org.team1540.robot2020.commands.drivetrain.PointToTarget;
import org.team1540.robot2020.utils.ChickenXboxController;
import org.team1540.robot2020.utils.NavX;
import org.team1540.rooster.wrappers.Limelight;

import java.util.function.BooleanSupplier;

public class ShooterLineUpSequence extends SequentialCommandGroup {
    Shooter shooter;
    PointToTarget pointingCommand;
    ShooterSpinUp shootingCommand;

    public ShooterLineUpSequence(double hoodAngle, double flywheelVelocity, NavX navx, DriveTrain driveTrain, ChickenXboxController driverController, Limelight limelight, Shooter shooter) {
        this.shooter = shooter;

        addCommands(parallel(
                this.pointingCommand = new PointToTarget(navx, driveTrain, driverController, limelight)
                ),
                this.shootingCommand = new ShooterSpinUp(shooter, 5000) //TODO: RPM constant
        );
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
