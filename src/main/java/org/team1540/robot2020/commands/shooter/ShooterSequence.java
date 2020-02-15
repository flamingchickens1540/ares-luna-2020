package org.team1540.robot2020.commands.shooter;

import edu.wpi.first.wpilibj2.command.*;
import org.team1540.robot2020.commands.funnel.Funnel;
import org.team1540.robot2020.commands.funnel.RunFunnel;
import org.team1540.robot2020.commands.hood.Hood;
import org.team1540.robot2020.commands.hood.HoodSetPosition;
import org.team1540.robot2020.commands.indexer.Indexer;
import org.team1540.robot2020.commands.intake.Intake;
import org.team1540.robot2020.commands.intake.IntakeRun;

public class ShooterSequence extends SequentialCommandGroup {
    public ShooterSequence(Intake intake, Funnel funnel, Indexer indexer, Shooter shooter, Hood hood) {
        addRequirements(intake, funnel, indexer, shooter, hood);
        addCommands(
                parallel(
                        new ShooterSpinUp(shooter, 5000),
                        new HoodSetPosition(hood, -100)
                ),
                deadline(new WaitCommand(2),
                        new InstantCommand(() -> indexer.setPercent(1)),
                        new RunFunnel(funnel, indexer),
                        new IntakeRun(intake)),
                new RunCommand(() -> {
                    intake.stop();
                    funnel.stop();
                    indexer.setPercent(0);
                    shooter.disableMotors();
                })
        );
    }
}
