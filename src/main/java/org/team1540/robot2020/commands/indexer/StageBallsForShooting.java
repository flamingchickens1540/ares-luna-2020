package org.team1540.robot2020.commands.indexer;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import org.team1540.robot2020.subsystems.Indexer;

public class StageBallsForShooting extends SequentialCommandGroup {
    public StageBallsForShooting(Indexer indexer) {
        addRequirements(indexer);
        addCommands(
                new MoveBallsToTop(indexer, Indexer.firstIndexingSpeed),
                new MoveBallsToTop(indexer, Indexer.secondIndexingSpeed)
        );
    }
}
