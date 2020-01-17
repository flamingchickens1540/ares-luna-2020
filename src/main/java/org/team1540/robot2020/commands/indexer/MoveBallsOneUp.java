package org.team1540.robot2020.commands.indexer;

import edu.wpi.first.wpilibj2.command.CommandBase;
import org.team1540.robot2020.subsystems.Indexer;

public class MoveBallsOneUp extends CommandBase {
    private Indexer indexer;

    public MoveBallsOneUp(Indexer indexer) {
        this.indexer = indexer;
    }
}
