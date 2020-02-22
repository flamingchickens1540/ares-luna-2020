package org.team1540.robot2020.commands;

import edu.wpi.first.wpilibj.geometry.Pose2d;
import edu.wpi.first.wpilibj.geometry.Rotation2d;
import edu.wpi.first.wpilibj.geometry.Translation2d;
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
                sequence(
//                        new TurnToAngle(driveTrain, localizationManager, this::getStartingPose, new Pose2d(0, 0, new Rotation2d(Math.PI/2)))
                        new ChickenRamseteCommand(
                                this::getStartingPose,
                                new Pose2d(0, 0, new Rotation2d(0)),
                                List.of(
                                        new Translation2d(2, 0.7)
                                ),
                                new Pose2d(4, 0.7, new Rotation2d(0)),
                                false, localizationManager, driveTrain
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