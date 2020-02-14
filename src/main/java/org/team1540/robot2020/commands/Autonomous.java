package org.team1540.robot2020.commands;

import edu.wpi.first.wpilibj.geometry.Pose2d;
import edu.wpi.first.wpilibj.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import org.team1540.robot2020.commands.drivetrain.ChickenRamseteCommand;
import org.team1540.robot2020.commands.drivetrain.DriveTrain;
import org.team1540.robot2020.commands.funnel.Funnel;
import org.team1540.robot2020.commands.indexer.Indexer;
import org.team1540.robot2020.commands.indexer.IndexerBallQueueSequence;
import org.team1540.robot2020.commands.intake.Intake;
import org.team1540.robot2020.commands.intake.IntakeRun;
import org.team1540.robot2020.commands.shooter.Shooter;
import org.team1540.robot2020.commands.shooter.ShooterSequence;
import org.team1540.robot2020.commands.shooter.ShooterSpinUp;

public class Autonomous extends SequentialCommandGroup {
    public Autonomous(DriveTrain driveTrain, Intake intake, Funnel funnel, Indexer indexer, Shooter shooter) {
        addCommands(
                new InstantCommand(() -> driveTrain.resetOdometry(new Pose2d())),
//                Move to trench while intaking and indexing
                race(
                        new IntakeRun(intake),
                        deadline(
                            new ChickenRamseteCommand(
//                                    y is just an estimate, see https://firstfrc.blob.core.windows.net/frc2020/PlayingField/LayoutandMarkingDiagram.pdf for actual measurements
                                    new Pose2d(0, 3, new Rotation2d(0)),
                                    driveTrain
                            ),
                            new IndexerBallQueueSequence(indexer, funnel)
                        )
                ),
//                Face goal and spin up shooter
                parallel(
                        new ShooterSpinUp(shooter),
                        new ChickenRamseteCommand(
                                new Pose2d(0, 3, new Rotation2d(Math.PI)),
                                driveTrain
                        )
                ),
                new ShooterSequence(intake, indexer, shooter)
        );
    }
}
