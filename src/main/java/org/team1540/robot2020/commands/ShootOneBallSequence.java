package org.team1540.robot2020.commands;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import org.team1540.robot2020.commands.indexer.MoveBallsToTop;
import org.team1540.robot2020.commands.indexer.MoveBallsUpOne;
import org.team1540.robot2020.commands.shooter.WaitForShooterUpToSpeed;
import org.team1540.robot2020.subsystems.Indexer;
import org.team1540.robot2020.subsystems.Shooter;

public class ShootOneBallSequence extends SequentialCommandGroup {
    public ShootOneBallSequence(Indexer indexer, Shooter shooter) {
        addCommands(
                new WaitForShooterUpToSpeed(shooter),
                new MoveBallsUpOne(indexer, 1.5),
                new WaitCommand(0.1),
                new InstantCommand(indexer::ballRemoved),
                new MoveBallsToTop(indexer)
        );
    }
}
