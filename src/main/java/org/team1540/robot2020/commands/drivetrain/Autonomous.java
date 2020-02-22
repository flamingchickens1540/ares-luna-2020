package org.team1540.robot2020.commands.drivetrain;

import edu.wpi.first.wpilibj.geometry.Pose2d;
import edu.wpi.first.wpilibj.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import org.team1540.robot2020.LocalizationManager;
import org.team1540.robot2020.commands.climber.Climber;
import org.team1540.robot2020.commands.funnel.Funnel;
import org.team1540.robot2020.commands.hood.Hood;
import org.team1540.robot2020.commands.hood.HoodZeroSequence;
import org.team1540.robot2020.commands.indexer.Indexer;
import org.team1540.robot2020.commands.indexer.IndexerBallQueueSequence;
import org.team1540.robot2020.commands.intake.Intake;
import org.team1540.robot2020.commands.intake.IntakePercent;
import org.team1540.robot2020.commands.shooter.Shooter;
import org.team1540.robot2020.utils.InstCommand;

import java.util.List;

public class Autonomous extends ParallelCommandGroup {
    private LocalizationManager localizationManager;
    private Pose2d startingPose;

    public Autonomous(DriveTrain driveTrain, Intake intake, Funnel funnel, Indexer indexer, Shooter shooter, Hood hood, Climber climber, LocalizationManager localizationManager) {
        this.localizationManager = localizationManager;
        addCommands(
                new InstCommand(() -> {
                    climber.zero();
                    climber.setRatchet(Climber.RatchetState.DISENGAGED);
                }),
                new HoodZeroSequence(hood),
                new IntakePercent(intake, 1),
                sequence(new IndexerBallQueueSequence(indexer, funnel, false),
                        new IndexerBallQueueSequence(indexer, funnel, false),
                        new IndexerBallQueueSequence(indexer, funnel, false),
                        new IndexerBallQueueSequence(indexer, funnel, false),
                        new IndexerBallQueueSequence(indexer, funnel, false)
                ),
                sequence(
//                        new PointToAngle(driveTrain, localizationManager, this::getStartingPose, new Pose2d(0, 0, new Rotation2d(Math.PI+Math.toRadians(5))), Math.toRadians(20)),
//                        new ChickenRamseteCommand(
//                                this::getStartingPose,
//                                List.of(
//                                        new Pose2d(0, 0, new Rotation2d(Math.PI)),
//                                        new Pose2d(-2, -.7, new Rotation2d(Math.PI)),
//                                        new Pose2d(-4, -.7, new Rotation2d(Math.PI))
//                                ),
//                                false, localizationManager, driveTrain
//                        )

                        new ChickenRamseteCommand(
                                this::getStartingPose,
                                List.of(
                                        new Pose2d(0, 0, new Rotation2d(0)),
                                        new Pose2d(4, 0, new Rotation2d(0))
                                ),
                                false, localizationManager, driveTrain
                        ),
                        new ChickenRamseteCommand(
                                this::getStartingPose,
                                List.of(
                                        new Pose2d(4, 0, new Rotation2d(0)),
                                        new Pose2d(0, 0, new Rotation2d(0))
                                ),
                                true, localizationManager, driveTrain
                        )
                )
        );
    }

    @Override
    public void initialize() {
        startingPose = localizationManager.odometryGetPose();
        super.initialize();
    }

    private Pose2d getStartingPose() {
        return startingPose;
    }
}