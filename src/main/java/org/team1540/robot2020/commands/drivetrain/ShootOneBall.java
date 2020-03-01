package org.team1540.robot2020.commands.drivetrain;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;
import org.team1540.robot2020.LocalizationManager;
import org.team1540.robot2020.commands.funnel.Funnel;
import org.team1540.robot2020.commands.funnel.FunnelRun;
import org.team1540.robot2020.commands.indexer.Indexer;
import org.team1540.robot2020.commands.indexer.IndexerBallsToTopFast;
import org.team1540.robot2020.commands.indexer.IndexerPercentToPosition;
import org.team1540.robot2020.commands.intake.Intake;
import org.team1540.robot2020.commands.intake.IntakeRun;
import org.team1540.robot2020.utils.InstCommand;

import java.util.function.BooleanSupplier;

public class ShootOneBall extends SequentialCommandGroup {

    public ShootOneBall(Intake intake, Funnel funnel, Indexer indexer, LocalizationManager localizationManager, BooleanSupplier isLinedUp) {
        addCommands(
                parallel(
                        race(
                                new IndexerBallsToTopFast(indexer, 0.2, 1),
                                new FunnelRun(funnel),
                                new IntakeRun(intake, 7000)
                        ),
                        new WaitUntilCommand(isLinedUp)
                ),
                new InstCommand(() -> localizationManager.ignoreLimelight(true)),
                race(
                        new IndexerPercentToPosition(indexer, () -> indexer.getPositionMeters() + SmartDashboard.getNumber("robotContainer/shootIndexDistance", 0.11), 1),
                        new FunnelRun(funnel),
                        new IntakeRun(intake, 7000)
                ),
                new InstCommand(() -> localizationManager.ignoreLimelight(false))
        );
    }

}