package org.team1540.robot2020.commands.hood;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;

public class HoodZeroSequence extends SequentialCommandGroup {
    public HoodZeroSequence(Hood hood) {
        addCommands(
                new HoodZero(hood, 0.45),
                new HoodMoveUntilNotPressed(hood, -0.25),
                new HoodZero(hood, 0.1)
        );
    }
}
