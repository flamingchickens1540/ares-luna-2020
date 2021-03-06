package org.team1540.robot2020.commands.shooter;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;
import org.team1540.robot2020.commands.indexer.Indexer;

import java.util.function.BooleanSupplier;

public class WaitThenShoot extends SequentialCommandGroup {
    public WaitThenShoot(BooleanSupplier waitCondition, Indexer indexer) {
        addCommands(
                new WaitUntilCommand(waitCondition),
                indexer.commandPercent(1)
        );
    }

}
