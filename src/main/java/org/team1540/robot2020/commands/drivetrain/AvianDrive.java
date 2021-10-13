package org.team1540.robot2020.commands.drivetrain;

import edu.wpi.first.wpilibj2.command.CommandBase;
import org.apache.log4j.Logger;
import org.team1540.robot2020.RobotContainer;
import org.team1540.robot2020.utils.Avian;

public class AvianDrive extends CommandBase {
    private static final Logger logger = Logger.getLogger(RobotContainer.class);
    private final DriveTrain driveTrain;
    private final Avian avian;
    private final double turnSpeed = 0.1;
    private final double driveSpeed = 0.1;

    public AvianDrive(DriveTrain driveTrain) {
        this.driveTrain = driveTrain;
        this.avian = new Avian();

        addRequirements(driveTrain);
    }

    private void setDriveTrain(double leftPercent, double rightPercent) {
        logger.info(leftPercent + ", " + rightPercent);
        // driveTrain.setPercent(leftPercent, rightPercent);
    }

    @Override
    public void execute() {
        // Stop if hands aren't detected
        if (avian.getDouble("avian/detected_hands") <= 2) {
            setDriveTrain(0, 0);
            return;
        }

        // Pinch to turn
        if (avian.getBooleanExclusive("avian/left_pinch")) {
            setDriveTrain(-turnSpeed, turnSpeed);
        } else if (avian.getBooleanExclusive("avian/right_pinch")) {
            setDriveTrain(turnSpeed, -turnSpeed);
        }

        // Fist to drive
        if (avian.getBooleanExclusive("avian/left_fist")) {
            setDriveTrain(driveSpeed, driveSpeed);
        } else if (avian.getBooleanExclusive("avian/right_fist")) {
            setDriveTrain(-driveSpeed, -driveSpeed);
        }
    }
}
