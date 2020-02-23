package org.team1540.robot2020.commands.funnel;

import edu.wpi.first.wpilibj2.command.CommandBase;

public class FunnelRun extends CommandBase {
    private Funnel funnel;

    public FunnelRun(Funnel funnel) {
        this.funnel = funnel;
        addRequirements(funnel);
    }

    @Override
    public void execute() {
        funnel.setPercent(0.1, 0.4);
    }

    @Override
    public void end(boolean interrupted) {
        funnel.setPercent(0);
    }
}
