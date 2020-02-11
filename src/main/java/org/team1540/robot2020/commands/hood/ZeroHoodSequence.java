package org.team1540.robot2020.commands.hood;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import org.team1540.robot2020.subsystems.Hood;

public class ZeroHoodSequence extends SequentialCommandGroup {
    public ZeroHoodSequence(Hood hood) {
        addCommands(
                new ZeroHood(hood, 0.45),
                new MoveUntilNotPressed(hood, -0.25),
                new ZeroHood(hood, 0.1)
        );
    }
}
