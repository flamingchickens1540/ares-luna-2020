package org.team1540.robot2020.commands.drivetrain;

import edu.wpi.first.wpilibj.geometry.Pose2d;
import edu.wpi.first.wpilibj.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import org.team1540.robot2020.LocalizationManager;
import org.team1540.robot2020.RamseteConfig;

import java.util.List;

public class AutoDriveTest extends SequentialCommandGroup {
    private LocalizationManager localizationManager;
    private Pose2d startingPose;

    public AutoDriveTest(DriveTrain driveTrain, LocalizationManager localizationManager) {
        this.localizationManager = localizationManager;
        addCommands(
                new ChickenRamseteCommand(
                        this::getStartingPose,
                        () -> List.of(
                                new Pose2d(2, -0.5, new Rotation2d(0))
                        ),
                        RamseteConfig.kMaxSpeedMetersPerSecond,
                        RamseteConfig.kMaxAccelerationMetersPerSecondSquared,
                        false, localizationManager, driveTrain
                )
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
