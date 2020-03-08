package org.team1540.robot2020.commands.shooter;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;
import org.team1540.robot2020.LocalizationManager;
import org.team1540.robot2020.commands.funnel.Funnel;
import org.team1540.robot2020.commands.funnel.FunnelRun;
import org.team1540.robot2020.commands.indexer.Indexer;
import org.team1540.robot2020.commands.indexer.IndexerBallsToTopFast;
import org.team1540.robot2020.commands.intake.Intake;
import org.team1540.robot2020.utils.InstCommand;

import java.util.function.BooleanSupplier;

public class ShootRapid extends SequentialCommandGroup {
    private LocalizationManager localizationManager;

    public ShootRapid(Intake intake, Funnel funnel, Indexer indexer, LocalizationManager localizationManager, BooleanSupplier isLinedUp) {
        this.localizationManager = localizationManager;
        addCommands(
                parallel(
                        race(
                                new IndexerBallsToTopFast(indexer, 0.2, 1),
                                new FunnelRun(funnel),
                                intake.commandPercent(1)
                        ),
                        new WaitUntilCommand(isLinedUp)
                ),
                new InstCommand(() -> localizationManager.ignoreLimelight(true)),
                race(
                        intake.commandPercent(1),
                        new FunnelRun(funnel),
                        indexer.commandPercent(1)
                )
        );
    }

    @Override
    public void end(boolean interrupted) {
        super.end(interrupted);

        localizationManager.ignoreLimelight(false);
    }
}
