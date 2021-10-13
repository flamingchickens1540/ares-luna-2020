package org.team1540.robot2020.commands.drivetrain;

import edu.wpi.first.wpilibj2.command.CommandBase;
import org.team1540.robot2020.utils.Avian;

public class AvianDrive extends CommandBase {
    private final DriveTrain driveTrain;
    private final Avian avian;

    private final double turnSpeed = 0.1;
    private final double driveSpeed = 0.1;

    public AvianDrive(DriveTrain driveTrain) {
        this.driveTrain = driveTrain;
        this.avian = new Avian();

        addRequirements(driveTrain);
    }

    @Override
    public void execute() {
        // Stop if hands aren't detected
        if (avian.getDouble("avian/detected_hands") <= 2) {
            driveTrain.setPercent(0, 0);
            return;
        }

        // Pinch to turn
        if (avian.getBooleanExclusive("avian/left_pinch")) {
            driveTrain.setPercent(-turnSpeed, turnSpeed);
        } else if (avian.getBooleanExclusive("avian/right_pinch")) {
            driveTrain.setPercent(turnSpeed, -turnSpeed);
        }

        // Fist to drive
        if (avian.getBooleanExclusive("avian/left_fist")) {
            driveTrain.setPercent(driveSpeed, driveSpeed);
        } else if (avian.getBooleanExclusive("avian/right_fist")) {
            driveTrain.setPercent(-driveSpeed, -driveSpeed);
        }
    }
}
