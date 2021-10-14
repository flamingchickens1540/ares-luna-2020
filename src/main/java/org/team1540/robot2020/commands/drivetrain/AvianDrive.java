package org.team1540.robot2020.commands.drivetrain;

import edu.wpi.first.wpilibj2.command.CommandBase;
import org.team1540.robot2020.commands.hood.Hood;
import org.team1540.robot2020.commands.hood.HoodZeroSequence;
import org.team1540.robot2020.commands.indexer.Indexer;
import org.team1540.robot2020.commands.shooter.Shooter;
import org.team1540.robot2020.utils.Avian;

public class AvianDrive extends CommandBase {
    private final DriveTrain driveTrain;
    private final Hood hood;
    private final Shooter shooter;
    private final Indexer indexer;

    private final Avian avian = new Avian();
    private final double turnSpeed = 0.25;
    private final double driveSpeed = 0.4;

    public AvianDrive(DriveTrain driveTrain, Hood hood, Shooter shooter, Indexer indexer) {
        this.driveTrain = driveTrain;
        this.hood = hood;
        this.shooter = shooter;
        this.indexer = indexer;
        addRequirements(driveTrain, hood);
    }

    @Override
    public void execute() {
        // Stop if hands aren't detected
        if (avian.getDouble("avian/detected_hands") < 2) {
            driveTrain.setPercent(0, 0);
            return;
        }

        // Drive states
        if (avian.getBooleanExclusive("avian/left_pinch")) {
            driveTrain.setPercent(-turnSpeed, turnSpeed);
        } else if (avian.getBooleanExclusive("avian/right_pinch")) {
            driveTrain.setPercent(turnSpeed, -turnSpeed);
        } else if (avian.getBooleanExclusive("avian/left_fist")) {
            driveTrain.setPercent(driveSpeed, driveSpeed);
        } else if (avian.getBooleanExclusive("avian/right_fist")) {
            driveTrain.setPercent(-driveSpeed, -driveSpeed);
        } else if (avian.getBooleanExclusive("avian/left_middle_finger") || avian.getBooleanExclusive("avian/right_middle_finger")) {
            driveTrain.setPercent(1, 1);
        } else if (avian.getBoolean("avian/left_middle_finger") && avian.getBoolean("avian/right_middle_finger")) {
            new HoodZeroSequence(hood).schedule();
            shooter.setVelocityRPM(7000);
            indexer.setPercent(0.5);
        } else { // Stop if no gestures are detected
            driveTrain.setPercent(0, 0);
            shooter.stop();
            indexer.setPercent(0);
        }
    }
}
