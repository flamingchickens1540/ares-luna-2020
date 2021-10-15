package org.team1540.robot2020.commands.avian;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import org.team1540.robot2020.commands.drivetrain.DriveTrain;
import org.team1540.robot2020.commands.funnel.Funnel;
import org.team1540.robot2020.commands.funnel.FunnelRun;
import org.team1540.robot2020.commands.indexer.Indexer;
import org.team1540.robot2020.commands.indexer.IndexerBallsToTopFast;
import org.team1540.robot2020.commands.intake.Intake;
import org.team1540.robot2020.commands.shooter.Shooter;
import org.team1540.robot2020.utils.Avian;

public class AvianDrive extends CommandBase {
    private final Avian avian;
    private final DriveTrain driveTrain;
    private final Intake intake;
    private final Funnel funnel;
    private final Indexer indexer;
    private final Shooter shooter;

    private final double turnSpeed = 0.25;
    private final double driveSpeed = 0.4;

    public AvianDrive(Avian avian, DriveTrain driveTrain, Intake intake, Funnel funnel, Indexer indexer, Shooter shooter) {
        this.avian = avian;
        this.driveTrain = driveTrain;
        this.intake = intake;
        this.funnel = funnel;
        this.indexer = indexer;
        this.shooter = shooter;
        SmartDashboard.putNumber("avian/shoot_speed", 0.7);
        addRequirements(driveTrain);
    }

    @Override
    public void execute() {
        if (avian.getBoolean("avian/right_index_finger")) {
            intake.setPercent(1);
            funnel.setPercent(1);
            indexer.setPercent(1);
            return;
        } else if (avian.getBoolean("avian/right_middle_finger") || avian.getBoolean("avian/left_middle_finger")) {
            shooter.setPercent(SmartDashboard.getNumber("avian/shoot_speed", 0.5));
            return;
        } else { // Stop if no gestures are detected
            intake.setPercent(0);
            funnel.setPercent(0);
            indexer.setPercent(0);
            shooter.setPercent(0);
        }

        // Stop if hands aren't detected
        if (avian.getDouble("avian/detected_hands") < 2) {
            driveTrain.setPercent(0, 0);
        } else if (avian.getBooleanExclusive("avian/left_pinch")) {
            driveTrain.setPercent(-turnSpeed, turnSpeed);
        } else if (avian.getBooleanExclusive("avian/right_pinch")) {
            driveTrain.setPercent(turnSpeed, -turnSpeed);
        } else if (avian.getBooleanExclusive("avian/left_fist")) {
            driveTrain.setPercent(driveSpeed, driveSpeed);
        } else if (avian.getBooleanExclusive("avian/right_fist")) {
            driveTrain.setPercent(-driveSpeed, -driveSpeed);
        } else {
            driveTrain.setPercent(0, 0);
        }
    }
}
