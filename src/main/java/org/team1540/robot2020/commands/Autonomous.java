package org.team1540.robot2020.commands;

import edu.wpi.first.wpilibj.geometry.Pose2d;
import edu.wpi.first.wpilibj.geometry.Rotation2d;
import edu.wpi.first.wpilibj.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import org.team1540.robot2020.commands.cargomech.CargoMechIntake;
import org.team1540.robot2020.shouldbeinrooster.ChickenRamseteCommand;
import org.team1540.robot2020.subsystems.CargoMech;
import org.team1540.robot2020.subsystems.DriveTrain;
import org.team1540.rooster.util.ChickenXboxController;

import java.util.List;

public class Autonomous extends SequentialCommandGroup {

    public Autonomous(DriveTrain driveTrain, CargoMech cargoMech, ChickenXboxController driver) {
        addCommands(
                new InstantCommand(() -> driveTrain.resetOdometry(new Pose2d())),
                new ChickenRamseteCommand(
                        List.of(
                                new Translation2d(0.5, 0.1),
                                new Translation2d(1, -0.1)
                        ),
                        new Pose2d(1.5, 0, new Rotation2d(0)),
                        driveTrain
                ),
                new CargoMechIntake(cargoMech),
                new ChickenRamseteCommand(
                        List.of(
                                new Translation2d(2, 0.1),
                                new Translation2d(2.5, 0.2)
                        ),
                        new Pose2d(3, 0, new Rotation2d(0)),
                        driveTrain
                )
//                new ChickenRamseteCommand(
//                        List.of(
//                                new Translation2d(-0.5, 0.1),
//                                new Translation2d(-1, -0.1)
//                        ),
//                        new Pose2d(-1.5, 0, new Rotation2d(0)),
//                        driveTrain,
//                        true
//                ),
//                new CargoMechIntake(cargoMech),
//                new ChickenRamseteCommand(
//                        List.of(
//                                new Translation2d(-2, 0.1),
//                                new Translation2d(-2.5, 0.2)
//                        ),
//                        new Pose2d(-3, 0, new Rotation2d(0)),
//                        driveTrain,
//                        true
//                )
        );
    }
}
