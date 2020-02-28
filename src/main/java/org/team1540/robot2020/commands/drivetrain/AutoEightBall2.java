package org.team1540.robot2020.commands.drivetrain;

import edu.wpi.first.wpilibj.geometry.Pose2d;
import edu.wpi.first.wpilibj.geometry.Rotation2d;
import edu.wpi.first.wpilibj.geometry.Transform2d;
import edu.wpi.first.wpilibj2.command.ConditionalCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import org.team1540.robot2020.LocalizationManager;
import org.team1540.robot2020.RamseteConfig;
import org.team1540.robot2020.commands.climber.Climber;
import org.team1540.robot2020.commands.funnel.Funnel;
import org.team1540.robot2020.commands.hood.Hood;
import org.team1540.robot2020.commands.hood.HoodZeroSequence;
import org.team1540.robot2020.commands.indexer.Indexer;
import org.team1540.robot2020.commands.indexer.IndexerBallQueueSequence;
import org.team1540.robot2020.commands.intake.Intake;
import org.team1540.robot2020.commands.intake.IntakeRun;
import org.team1540.robot2020.commands.shooter.Shooter;
import org.team1540.robot2020.utils.ChickenXboxController;
import org.team1540.robot2020.utils.InstCommand;

import java.util.List;

import static org.team1540.robot2020.utils.LoopCommand.loop;

public class AutoEightBall2 extends SequentialCommandGroup {
    private LocalizationManager localizationManager;
    private Pose2d startingPose;

    public AutoEightBall2(DriveTrain driveTrain, Intake intake, Funnel funnel, Indexer indexer, Shooter shooter, Hood hood, Climber climber, LocalizationManager localizationManager, ChickenXboxController driverController) {
        this.localizationManager = localizationManager;
        addCommands(
                new InstCommand(() -> {
                    climber.zero();
                    climber.setRatchet(Climber.RatchetState.DISENGAGED);
                }),
                new ConditionalCommand(
                        sequence(
                                new HoodZeroSequence(hood).asProxy(),
                                new InstCommand(() -> hood.zeroFlag = false)
                        ),
                        new InstCommand(),
                        () -> hood.zeroFlag
                ),
                new AutoShootNBalls(3, driveTrain, intake, funnel, indexer, shooter, hood, localizationManager, driverController, 3),
                race(
                        new IntakeRun(intake, 7000),
                        loop(new IndexerBallQueueSequence(indexer, funnel, false)),
                        sequence(
                                new PointToRotation(driveTrain, localizationManager, new Pose2d(0, 0, new Rotation2d(Math.toRadians(-120))), Math.toRadians(20)),
                                new ChickenRamseteCommand(
                                        this::getStartingPose,
                                        () -> List.of(
                                                new Pose2d(0, 0, new Rotation2d(Math.toRadians(-120))),
                                                new Pose2d(-2, -.8, new Rotation2d(Math.PI)),
                                                new Pose2d(-4, -.8, new Rotation2d(Math.PI))
                                        ),
                                        1.75,
                                        RamseteConfig.kMaxAccelerationMetersPerSecondSquared,
                                        false, localizationManager, driveTrain
                                ),
                                new ChickenRamseteCommand(
                                        this::getStartingPose,
                                        () -> List.of(
                                                new Pose2d(-4, -.8, new Rotation2d(Math.PI)),
                                                new Pose2d(-2, 0.5, new Rotation2d(Math.toRadians(-150)))
                                        ),
                                        2.5,
                                        RamseteConfig.kMaxAccelerationMetersPerSecondSquared,
                                        true, localizationManager, driveTrain
                                ),
                                new PointToRotation(driveTrain, localizationManager, new Pose2d(0, 0, new Rotation2d(Math.toRadians(120))), Math.toRadians(4)),
                                new ChickenRamseteCommand(
                                        this::getStartingPose,
                                        () -> List.of(
                                                new Pose2d(-1.7, 0.5, new Rotation2d(Math.toRadians(130))),
                                                new Pose2d(-1.7, 0.5, new Rotation2d(Math.toRadians(130))).plus(new Transform2d(new Pose2d(), new Pose2d(0.75, 0, new Rotation2d())))
                                        ),
                                        RamseteConfig.kMaxSpeedMetersPerSecond,
                                        RamseteConfig.kMaxAccelerationMetersPerSecondSquared,
                                        false, localizationManager, driveTrain
                                ),
                                new ChickenRamseteCommand(
                                        this::getStartingPose,
                                        () -> List.of(
                                                new Pose2d(-1.7, 0.5, new Rotation2d(Math.toRadians(130))).plus(new Transform2d(new Pose2d(), new Pose2d(0.65, 0, new Rotation2d()))),
                                                new Pose2d(-0.5, 0, new Rotation2d(Math.PI))
                                        ),
                                        2.5,
                                        RamseteConfig.kMaxAccelerationMetersPerSecondSquared,
                                        true, localizationManager, driveTrain
                                )
                        )
                ),
                new AutoShootNBalls(5, driveTrain, intake, funnel, indexer, shooter, hood, localizationManager, driverController, 6)
        );
    }

    @Override
    public void initialize() {
        Pose2d startingPose = localizationManager.odometryGetPose();
        this.startingPose = new Pose2d(startingPose.getTranslation(), new Rotation2d());
        super.initialize();
    }

    private Pose2d getStartingPose() {
        return startingPose;
    }
}