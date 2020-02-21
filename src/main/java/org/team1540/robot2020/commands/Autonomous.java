package org.team1540.robot2020.commands;

import edu.wpi.first.wpilibj.geometry.Pose2d;
import edu.wpi.first.wpilibj.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import org.team1540.robot2020.LocalizationManager;
import org.team1540.robot2020.commands.drivetrain.ChickenRamseteCommand;
import org.team1540.robot2020.commands.drivetrain.DriveTrain;
import org.team1540.robot2020.commands.drivetrain.TurnToAngle;
import org.team1540.robot2020.commands.funnel.Funnel;
import org.team1540.robot2020.commands.indexer.Indexer;
import org.team1540.robot2020.commands.indexer.IndexerBallQueueSequence;
import org.team1540.robot2020.commands.intake.Intake;
import org.team1540.robot2020.commands.intake.IntakeRun;
import org.team1540.robot2020.commands.shooter.Shooter;
import org.team1540.robot2020.utils.InstCommand;

public class Autonomous extends SequentialCommandGroup {
    public Autonomous(DriveTrain driveTrain, Intake intake, Funnel funnel, Indexer indexer, Shooter shooter, LocalizationManager localizationManager) {
        addCommands(
                new InstCommand(driveTrain::resetEncoders),
                new InstCommand(() -> localizationManager.resetOdometry(new Pose2d())),

//                new ShooterSpinUp(shooter),
//                new ShooterSequence(intake, indexer, shooter),
                new TurnToAngle(driveTrain, localizationManager, Math.PI),
                deadline(
                        new ChickenRamseteCommand(
//                                    y is just an estimate, see https://firstfrc.blob.core.windows.net/frc2020/PlayingField/LayoutandMarkingDiagram.pdf for actual measurements
                                new Pose2d(3, 0, new Rotation2d()),
                                localizationManager,
                                driveTrain
                        ),
                        new IntakeRun(intake)
//                        new IndexerBallQueueSequence(indexer, funnel)
                ),
                new TurnToAngle(driveTrain, localizationManager, 0)
//                new ShooterSpinUp(shooter),
//                new ShooterSequence(intake, indexer, shooter)
        );
    }
}