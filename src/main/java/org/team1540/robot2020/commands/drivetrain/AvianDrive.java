package org.team1540.robot2020.commands.drivetrain;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import org.team1540.robot2020.commands.funnel.Funnel;
import org.team1540.robot2020.commands.indexer.Indexer;
import org.team1540.robot2020.commands.intake.Intake;
import org.team1540.robot2020.commands.shooter.Shooter;

public class AvianDrive extends CommandBase {
    private final DriveTrain driveTrain;
    private final Intake intake;
    private final Funnel funnel;
    private final Indexer indexer;
    private final Shooter shooter;

    private final double defaultShootSpeed = 0.7;
    private final double defaultDriveSpeed = 0.4;
    private final double defaultTurnSpeed = 0.2;

    public AvianDrive(DriveTrain driveTrain, Intake intake, Funnel funnel, Indexer indexer, Shooter shooter) {
        this.driveTrain = driveTrain;
        this.intake = intake;
        this.funnel = funnel;
        this.indexer = indexer;
        this.shooter = shooter;
        SmartDashboard.putNumber("avian/drive_speed", defaultDriveSpeed);
        SmartDashboard.putNumber("avian/turn_speed", defaultTurnSpeed);
        SmartDashboard.putNumber("avian/shoot_speed", defaultShootSpeed);
        addRequirements(driveTrain);
    }

    private void stop() {
        driveTrain.setPercent(0, 0);
        intake.setPercent(0);
        funnel.setPercent(0);
        indexer.setPercent(0);
        shooter.setPercent(0);
    }

    @Override
    public void execute() {
        // Stop if 2 hands aren't detected
        if (SmartDashboard.getNumber("avian/detected_hands", 0) < 2) {
            stop();
            return;
        }

        double driveSpeed = SmartDashboard.getNumber("avian/drive_speed", defaultDriveSpeed);
        double turnSpeed = SmartDashboard.getNumber("avian/turn_speed", defaultTurnSpeed);
        double shootSpeed = SmartDashboard.getNumber("avian/shoot_speed", defaultShootSpeed);

        boolean leftPinch = SmartDashboard.getBoolean("avian/left_pinch", false);
        boolean rightPinch = SmartDashboard.getBoolean("avian/right_pinch", false);
        boolean leftIndex = SmartDashboard.getBoolean("avian/left_index_finger", false);
        boolean rightIndex = SmartDashboard.getBoolean("avian/right_index_finger", false);
        boolean leftMiddle = SmartDashboard.getBoolean("avian/left_middle_finger", false);
        boolean rightMiddle = SmartDashboard.getBoolean("avian/right_middle_finger", false);

        // Drive
        if (leftPinch && rightPinch) {
            driveTrain.setPercent(driveSpeed, driveSpeed);
        } else if (leftPinch && !rightPinch) {
            driveTrain.setPercent(-turnSpeed, turnSpeed);
        } else if (!leftPinch && rightPinch) {
            driveTrain.setPercent(turnSpeed, -turnSpeed);
        } else if (leftIndex || rightIndex) {
            shooter.setPercent(shootSpeed);
        } else if (leftMiddle || rightMiddle) {
            intake.setPercent(1);
            funnel.setPercent(1);
            indexer.setPercent(1);
        } else {
            stop();
        }
    }
}
