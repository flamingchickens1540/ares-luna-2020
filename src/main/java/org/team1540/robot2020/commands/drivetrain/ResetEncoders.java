package org.team1540.robot2020.commands.drivetrain;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import org.team1540.robot2020.subsystems.DriveTrain;

public class ResetEncoders extends InstantCommand {
    private DriveTrain driveTrain;

    public ResetEncoders(DriveTrain driveTrain) {
        this.driveTrain = driveTrain;
    }
    @Override
    public void initialize() {
        driveTrain.resetEncoders();
    }
}
