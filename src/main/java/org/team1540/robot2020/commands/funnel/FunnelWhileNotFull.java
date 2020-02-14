package org.team1540.robot2020.commands.funnel;

import edu.wpi.first.wpilibj2.command.CommandBase;
import org.team1540.robot2020.commands.indexer.Indexer;

public class FunnelWhileNotFull extends CommandBase {
    private Funnel funnel;
    private Indexer indexer;

    public FunnelWhileNotFull(Funnel funnel, Indexer indexer) {
        this.funnel = funnel;
        this.indexer = indexer;
        addRequirements(funnel);
    }

    @Override
    public void execute() {
//        if (indexer.isFull) {
//            funnel.setPercent(0);
//        } else {
//            funnel.setPercent(0.5, 0.1);
//        }
        funnel.setPercent(0.3);
    }

    @Override
    public void end(boolean interrupted) {
        funnel.setPercent(0);
    }
}
