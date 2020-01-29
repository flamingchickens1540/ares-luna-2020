package org.team1540.robot2020.commands;

import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import org.team1540.robot2020.commands.indexer.MoveBallsToTop;
import org.team1540.robot2020.commands.shooter.SpinUpShooter;
import org.team1540.robot2020.subsystems.Indexer;
import org.team1540.robot2020.subsystems.Shooter;

public class PrimeShooterSequence extends ParallelCommandGroup {
    public PrimeShooterSequence(Indexer indexer, Shooter shooter) {
        addCommands(
                new MoveBallsToTop(indexer),
                new SpinUpShooter(shooter, 2000)
        );
    }
}
