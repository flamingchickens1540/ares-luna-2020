package org.team1540.robot2020.commands.hood;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;

public class HoodZeroSequence extends SequentialCommandGroup {
    public HoodZeroSequence(Hood hood) {
        addCommands(
                new HoodMoveUntilPressed(hood, 0.45, true),
                new HoodMoveUntilPressed(hood, -0.25, false),
                new HoodMoveUntilPressed(hood, 0.1, true),
                new InstantCommand(hood::zero)
        );
    }
}
