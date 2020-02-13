package org.team1540.robot2020.shouldbeinrooster;

import edu.wpi.first.wpilibj.trajectory.Trajectory;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import org.team1540.robot2020.subsystems.DriveTrain;

public class ChickenRamseteCommand extends SequentialCommandGroup {
    public ChickenRamseteCommand(Trajectory trajectory, DriveTrain driveTrain) {
        addCommands(
                new BaseChickenRamseteCommand(trajectory, driveTrain),
                new InstCommand(driveTrain::stop)
        );
    }
}
