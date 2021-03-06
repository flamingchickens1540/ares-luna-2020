package org.team1540.robot2020.commands.indexer;

import edu.wpi.first.wpilibj2.command.CommandBase;
import org.team1540.robot2020.utils.ChickenXboxController;

public class IndexerManualControl extends CommandBase {
    private Indexer indexer;
    private ChickenXboxController.Axis joystickAxis;

    public IndexerManualControl(Indexer indexer, ChickenXboxController.Axis joystickAxis) {
        this.indexer = indexer;
        this.joystickAxis = joystickAxis;
        addRequirements(indexer);
    }

    @Override
    public void execute() {
        indexer.setPercent(joystickAxis.value());
    }
}
