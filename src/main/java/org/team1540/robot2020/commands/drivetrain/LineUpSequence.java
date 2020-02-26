package org.team1540.robot2020.commands.drivetrain;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;
import org.team1540.robot2020.LocalizationManager;
import org.team1540.robot2020.commands.hood.Hood;
import org.team1540.robot2020.commands.indexer.Indexer;
import org.team1540.robot2020.commands.shooter.Shooter;
import org.team1540.robot2020.commands.shooter.ShooterSetVelocityContinuous;
import org.team1540.robot2020.utils.ChickenXboxController;
import org.team1540.robot2020.utils.InstCommand;

public class LineUpSequence extends SequentialCommandGroup {

    public LineUpSequence(DriveTrain driveTrain, Indexer indexer, Shooter shooter, Hood hood, ChickenXboxController driverController, LocalizationManager localizationManager, boolean inCommandGroup, boolean useThrottle) {

        addCommands(
                parallel(
                        sequence(
                                new InstCommand(localizationManager::selectTarget),
                                new WaitUntilCommand(localizationManager::useLidarForDistanceEst), // TODO: should this really be the trigger?
                                new InstCommand(localizationManager::selectTarget)
                        ),
                        new ShooterSetVelocityContinuous(shooter, localizationManager::getShooterRPMForSelectedGoal)
                )
        );
    }
}
