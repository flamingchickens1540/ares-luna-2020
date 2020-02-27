package org.team1540.robot2020.commands.drivetrain;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.ParallelRaceGroup;
import org.team1540.robot2020.LocalizationManager;
import org.team1540.robot2020.commands.funnel.Funnel;
import org.team1540.robot2020.commands.hood.Hood;
import org.team1540.robot2020.commands.indexer.Indexer;
import org.team1540.robot2020.commands.intake.Intake;
import org.team1540.robot2020.commands.shooter.Shooter;
import org.team1540.robot2020.commands.shooter.ShooterSetVelocityContinuous;
import org.team1540.robot2020.utils.ChickenXboxController;

import static org.team1540.robot2020.utils.LoopCommand.loop;

public class AutoShootNBalls extends ParallelRaceGroup {

    public AutoShootNBalls(int balls, DriveTrain driveTrain, Intake intake, Funnel funnel, Indexer indexer, Shooter shooter, Hood hood, LocalizationManager localizationManager, ChickenXboxController driverController, int timeout) {
        Command lineUpSequence = parallel(
                new ShooterSetVelocityContinuous(shooter, localizationManager::getShooterRPMForSelectedGoal),
                new LineUpSequence(driveTrain, indexer, shooter, hood, driverController, localizationManager, false, false)
        );

        addCommands(
                lineUpSequence, // todo: should start spinning up earlier before shooting
                loop(
                        new ShootOneBall(intake, funnel, indexer, localizationManager, localizationManager::isLinedUp),
                        balls
                ).withTimeout(timeout) // todo: tune timeout
        );
    }
}
