package org.team1540.robot2020.commands.shooter;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import org.team1540.robot2020.commands.indexer.MoveBallsToTop;
import org.team1540.robot2020.commands.indexer.MoveBallsUpOne;
import org.team1540.robot2020.subsystems.Indexer;
import org.team1540.robot2020.subsystems.Intake;
import org.team1540.robot2020.subsystems.Shooter;

public class ShootSequence extends SequentialCommandGroup {
    private Shooter shooter;
    public ShootSequence(Intake intake, Indexer indexer, Shooter shooter) {
        this.shooter = shooter;
        addRequirements(indexer);
        addCommands(
                new ParallelCommandGroup(
                        new SpinUpShooter(shooter, 2000),
                        new MoveBallsToTop(indexer)
                ),
                // todo ball widths should be a tuning value
                new MoveBallsUpOne(indexer, 1.5),
                new WaitCommand(0.1),
                // todo this should go before the waitcommand:
                new InstantCommand(indexer::ballRemoved)
        );
    }

    @Override
    public void end(boolean interrupted) {
        shooter.stop();
    }
}