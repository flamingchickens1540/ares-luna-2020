package org.team1540.robot2020.commands.drivetrain;

import edu.wpi.first.wpilibj2.command.ParallelRaceGroup;
import org.team1540.robot2020.LocalizationManager;
import org.team1540.robot2020.commands.climber.Climber;
import org.team1540.robot2020.commands.funnel.Funnel;
import org.team1540.robot2020.commands.hood.Hood;
import org.team1540.robot2020.commands.indexer.Indexer;
import org.team1540.robot2020.commands.intake.Intake;
import org.team1540.robot2020.commands.shooter.Shooter;
import org.team1540.robot2020.utils.ChickenXboxController;

public class AutoShootThreeBalls extends ParallelRaceGroup {

    public AutoShootThreeBalls(DriveTrain driveTrain, Intake intake, Funnel funnel, Indexer indexer, Shooter shooter, Hood hood, Climber climber, LocalizationManager localizationManager, ChickenXboxController driverController) {
        ShooterLineUpSequence shooterLineUpSequence = new ShooterLineUpSequence(driveTrain, indexer, shooter, hood, driverController, localizationManager, true);

        addCommands(
                shooterLineUpSequence, // todo: should start spinning up earlier before shooting
                sequence(
                        new ShootOneBall(intake, funnel, indexer, localizationManager, shooterLineUpSequence::isLinedUp),
                        new ShootOneBall(intake, funnel, indexer, localizationManager, shooterLineUpSequence::isLinedUp),
                        new ShootOneBall(intake, funnel, indexer, localizationManager, shooterLineUpSequence::isLinedUp)
                )
        );
    }
}
