package org.team1540.robot2020.commands.indexer;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import org.team1540.robot2020.subsystems.Indexer;

public class StageBallsForShooting extends SequentialCommandGroup {

    private double goal;

    public StageBallsForShooting(Indexer indexer) {
        addRequirements(indexer);
        addCommands(
                new IndexerMoveToPosition(indexer, () -> indexer.getPositionMeters() + goal, 0.5, 0.05),
                new IndexerBallsToTop(indexer, 0.1)
        );

        SmartDashboard.putNumber("indexer/stageDistance", 0.82);
    }

    @Override
    public void initialize() {
        super.initialize();

        goal = SmartDashboard.getNumber("indexer/stageDistance", 0);
    }
}
