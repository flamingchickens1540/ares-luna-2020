package org.team1540.robot2020.commands.avian;

import edu.wpi.first.wpilibj2.command.ConditionalCommand;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import org.team1540.robot2020.LocalizationManager;
import org.team1540.robot2020.commands.funnel.Funnel;
import org.team1540.robot2020.commands.funnel.FunnelRun;
import org.team1540.robot2020.commands.indexer.Indexer;
import org.team1540.robot2020.commands.indexer.IndexerBallsToTopFast;
import org.team1540.robot2020.commands.intake.Intake;
import org.team1540.robot2020.commands.shooter.Shooter;
import org.team1540.robot2020.utils.Avian;
import org.team1540.robot2020.utils.InstCommand;

public class AvianShoot extends SequentialCommandGroup {
    private final LocalizationManager localizationManager;

    public AvianShoot(Intake intake, Funnel funnel, Indexer indexer, Shooter shooter, Avian avian, LocalizationManager localizationManager) {
        this.localizationManager = localizationManager;
        addCommands(
                new ConditionalCommand(
                        sequence(
                                new InstCommand(() -> shooter.setPercent(0.25), shooter),
                                new WaitCommand(5),
                                parallel(
                                        race(
                                                new IndexerBallsToTopFast(indexer, 0.2, 1),
                                                new FunnelRun(funnel),
                                                intake.commandPercent(1)
                                        )
                                ),
                                new InstCommand(() -> localizationManager.ignoreLimelight(true)),
                                race(
                                        intake.commandPercent(1),
                                        new FunnelRun(funnel),
                                        indexer.commandPercent(1)
                                )
                        ),
                        new InstCommand(() -> {
                            shooter.setPercent(0);
                            indexer.setPercent(0);
                            funnel.setPercent(0);
                        }),
                        () -> avian.getBooleanExclusive("avian/left_middle_finger")
                )
        );
    }

    @Override
    public void end(boolean interrupted) {
        super.end(interrupted);

        localizationManager.ignoreLimelight(false);
    }
}
