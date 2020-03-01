package org.team1540.robot2020.commands.drivetrain;

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
import org.team1540.robot2020.commands.indexer.Indexer;
import org.team1540.robot2020.commands.intake.Intake;
import org.team1540.robot2020.commands.shooter.Shooter;
import org.team1540.robot2020.utils.ChickenXboxController;

import java.util.List;

public class AutoThreeBallDrive extends SequentialCommandGroup {

    private LocalizationManager localizationManager;
    private Pose2d startingPose;

    public AutoThreeBallDrive(DriveTrain driveTrain, Intake intake, Funnel funnel, Indexer indexer, Shooter shooter, Hood hood, Climber climber, LocalizationManager localizationManager, ChickenXboxController driverController) {
        this.localizationManager = localizationManager;
        SmartDashboard.putBoolean("AutoThreeBall/DriveForwards", false);
        addCommands(new ConditionalCommand(new ChickenRamseteCommand(
                this::getStartingPose,
                () -> List.of(
                        new Pose2d(0, 0, new Rotation2d(localizationManager.getYawRadians())),
                        new Pose2d(1.5, 0, new Rotation2d(0))
                ),
                RamseteConfig.kMaxSpeedMetersPerSecond,
                RamseteConfig.kMaxAccelerationMetersPerSecondSquared,
                false, localizationManager, driveTrain),
                new ChickenRamseteCommand(
                        this::getStartingPose,
                        () -> List.of(
                                new Pose2d(0, 0, new Rotation2d(localizationManager.getYawRadians())),
                                new Pose2d(-1.5, 0, new Rotation2d(0))
                        ),
                        RamseteConfig.kMaxSpeedMetersPerSecond,
                        RamseteConfig.kMaxAccelerationMetersPerSecondSquared,
                        true, localizationManager, driveTrain),
                () -> SmartDashboard.getBoolean("AutoThreeBall/DriveForwards", false)
        ));
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
