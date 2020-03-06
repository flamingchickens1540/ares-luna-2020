package org.team1540.robot2020.commands.drivetrain;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.geometry.Pose2d;
import edu.wpi.first.wpilibj.geometry.Rotation2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
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

public class AutoSixBall extends SequentialCommandGroup {
    private LocalizationManager localizationManager;
    private Pose2d startingPose;

    public AutoSixBall(DriveTrain driveTrain, Intake intake, Funnel funnel, Indexer indexer, Shooter shooter, Hood hood, Climber climber, LocalizationManager localizationManager, ChickenXboxController driverController) {
        this.localizationManager = localizationManager;
        SmartDashboard.putBoolean("AutoSixBall/driveUpClose", true);
        addCommands(
                new InstCommand(() -> {
                    SmartDashboard.putString("AutoSelector/SelectedAuto", "SixBall");
                    climber.zero();
                    climber.setRatchet(Climber.RatchetState.DISENGAGED);
                }),
                driveTrain.commandStop(),
                new ConditionalCommand(
                        sequence(
                                new HoodZeroSequence(hood).asProxy(),
                                new InstCommand(() -> hood.zeroFlag = false)
                        ),
                        new InstCommand(),
                        () -> true
                ),
                new AutoShootNBalls(3, driveTrain, intake, funnel, indexer, shooter, hood, localizationManager, driverController, 3),
                race(
//                                new IntakePercent(intake, 1),
                        new IntakeRun(intake, 7000),
                        loop(new IndexerBallQueueSequence(indexer, funnel, false)),
                        sequence(
                                new PointToRotation(driveTrain, localizationManager, new Pose2d(0, 0, new Rotation2d(Math.toRadians(-170))), Math.toRadians(10)),
                                new ChickenRamseteCommand(
                                        this::getStartingPose,
                                        () -> List.of(
                                                new Pose2d(0, 0, new Rotation2d(Math.toRadians(-170))),
                                                new Pose2d(-2, -.6, new Rotation2d(Math.PI)),
                                                new Pose2d(-4, -.6, new Rotation2d(Math.PI))
                                        ),
//                                        RamseteConfig.kMaxSpeedMetersPerSecond,
                                        1.25,
                                        RamseteConfig.kMaxAccelerationMetersPerSecondSquared,
                                        false, localizationManager, driveTrain
                                ),
                                new ConditionalCommand(sequence(
                                        new PointToRotation(driveTrain, localizationManager, new Pose2d(-4, -.8, new Rotation2d(Math.toRadians(10))), Math.toRadians(10)),
                                        new ChickenRamseteCommand(
                                                this::getStartingPose,
                                                () -> List.of(
                                                        new Pose2d(-4, -.8, new Rotation2d(Math.toRadians(10))),
                                                        new Pose2d(0, 0, new Rotation2d(Math.toRadians(10)))
                                                ),
                                                RamseteConfig.kMaxSpeedMetersPerSecond,
//                                1.25,
                                                RamseteConfig.kMaxAccelerationMetersPerSecondSquared,
                                                false, localizationManager, driveTrain
                                        )
                                ), new InstCommand(), () -> SmartDashboard.getBoolean("AutoSixBall/driveUpClose", true))
                        )
                ),
                new AutoShootNBalls(3, driveTrain, intake, funnel, indexer, shooter, hood, localizationManager, driverController, 6)
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
