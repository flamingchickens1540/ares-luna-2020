package org.team1540.robot2020.commands.indexer;

import edu.wpi.first.wpilibj2.command.ConditionalCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import org.team1540.robot2020.subsystems.Indexer;
import org.team1540.robot2020.utils.InstCommand;

public class StageBallsForShooting extends SequentialCommandGroup {
    public StageBallsForShooting(Indexer indexer) {
        addRequirements(indexer);
        addCommands(
                new ConditionalCommand(
                        new InstCommand(),
                        new MoveBallsToTop(indexer, Indexer.firstIndexingSpeed),
                        indexer::getShooterStagedSensor
                ),
                new MoveBallsToTop(indexer, Indexer.secondIndexingSpeed)
        );
    }
}
