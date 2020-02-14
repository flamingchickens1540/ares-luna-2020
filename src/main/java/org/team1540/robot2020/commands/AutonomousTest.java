package org.team1540.robot2020.commands;

import edu.wpi.first.wpilibj.geometry.Pose2d;
import edu.wpi.first.wpilibj.geometry.Rotation2d;
import edu.wpi.first.wpilibj.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import org.team1540.robot2020.commands.drivetrain.ChickenRamseteCommand;
import org.team1540.robot2020.commands.drivetrain.DriveTrain;

import java.util.List;

public class AutonomousTest extends SequentialCommandGroup {

    public AutonomousTest(DriveTrain driveTrain) {
        addCommands(
                new InstantCommand(() -> driveTrain.resetOdometry(new Pose2d())),
                new ChickenRamseteCommand(
                        new Pose2d(1.5, 0, new Rotation2d(0)),
                        List.of(
                                new Translation2d(0.5, 0.1),
                                new Translation2d(1, -0.1)
                        ),
                        driveTrain
                ),
                new ChickenRamseteCommand(
                        new Pose2d(3, 0, new Rotation2d(0)),
                        List.of(
                                new Translation2d(2, 0.1),
                                new Translation2d(2.5, 0.2)
                        ),
                        driveTrain
                )
//                new ChickenRamseteCommand(
//                        new Pose2d(-1.5, 0, new Rotation2d(0)),
//                        List.of(
//                                new Translation2d(-0.5, 0.1),
//                                new Translation2d(-1, -0.1)
//                        ),
//                        driveTrain,
//                        true
//                ),
//                new ChickenRamseteCommand(
//                        new Pose2d(-3, 0, new Rotation2d(0)),
//                        List.of(
//                                new Translation2d(-2, 0.1),
//                                new Translation2d(-2.5, 0.2)
//                        ),
//                        driveTrain,
//                        true
//                )
        );
    }
}