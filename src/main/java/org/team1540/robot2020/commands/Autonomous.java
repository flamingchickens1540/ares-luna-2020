package org.team1540.robot2020.commands;

import edu.wpi.first.wpilibj.geometry.Pose2d;
import edu.wpi.first.wpilibj.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import org.team1540.robot2020.LocalizationManager;
import org.team1540.robot2020.commands.climber.Climber;
import org.team1540.robot2020.commands.drivetrain.ChickenRamseteCommand;
import org.team1540.robot2020.commands.drivetrain.DriveTrain;
import org.team1540.robot2020.commands.funnel.Funnel;
import org.team1540.robot2020.commands.hood.Hood;
import org.team1540.robot2020.commands.hood.HoodZeroSequence;
import org.team1540.robot2020.commands.indexer.Indexer;
import org.team1540.robot2020.commands.intake.Intake;
import org.team1540.robot2020.commands.shooter.Shooter;
import org.team1540.robot2020.utils.InstCommand;

public class Autonomous extends ParallelCommandGroup {
    private LocalizationManager localizationManager;
    private Pose2d startingPose;

    public Autonomous(DriveTrain driveTrain, Intake intake, Funnel funnel, Indexer indexer, Shooter shooter, Hood hood, Climber climber, LocalizationManager localizationManager) {
        this.localizationManager = localizationManager;
        addCommands(
                new InstCommand(() -> {
                    localizationManager.resetOdometry(new Pose2d());
                    climber.zero();
                    climber.setRatchet(Climber.RatchetState.DISENGAGED);

                }),
                new HoodZeroSequence(hood),
                sequence(
                    new ChickenRamseteCommand(
                            this::getStartingPose,
                            new Pose2d(0, 0, new Rotation2d(0)),
                            new Pose2d(1, 0, new Rotation2d(0)),
                            false,
                            localizationManager,
                            driveTrain
                    ),
                    new ChickenRamseteCommand(
                            this::getStartingPose,
                            new Pose2d(1, 0, new Rotation2d(0)),
                            new Pose2d(2, 0, new Rotation2d(0)),
                            false,
                            localizationManager,
                            driveTrain
                    )
    //                new ShooterSpinUp(shooter),
    //                new ShooterSequence(intake, indexer, shooter),
    //                new TurnToAngle(driveTrain, localizationManager, Math.PI),
    //                deadline(
    //                        new ChickenRamseteCommand(
    //                                    y is just an estimate, see https://firstfrc.blob.core.windows.net/frc2020/PlayingField/LayoutandMarkingDiagram.pdf for actual measurements
    //                                RamseteTranslator.translatePose(new Pose2d(2, 0, new Rotation2d())),
    //                                localizationManager,
    //                                driveTrain
    //                        ),
    //                        new IntakeRun(intake)
    //                        new IndexerBallQueueSequence(indexer, funnel)
    //                ),
    //                new TurnToAngle(driveTrain, localizationManager, 0)
    //                new ShooterSpinUp(shooter),
    //                new ShooterSequence(intake, indexer, shooter)
                )
        );
    }

    @Override
    public void initialize() {
        super.initialize();
        startingPose = localizationManager.odometryGetPose();
    }

    private Pose2d getStartingPose() {
        return startingPose;
    }
}