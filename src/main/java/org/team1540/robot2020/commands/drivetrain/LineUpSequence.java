package org.team1540.robot2020.commands.drivetrain;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;
import org.team1540.robot2020.LocalizationManager;
import org.team1540.robot2020.commands.hood.Hood;
import org.team1540.robot2020.commands.hood.HoodSetPositionContinuous;
import org.team1540.robot2020.commands.indexer.Indexer;
import org.team1540.robot2020.commands.shooter.Shooter;
import org.team1540.robot2020.commands.shooter.ShooterSetVelocityContinuous;
import org.team1540.robot2020.utils.ChickenXboxController;
import org.team1540.robot2020.utils.InstCommand;

public class LineUpSequence extends SequentialCommandGroup {
    private LocalizationManager localizationManager;
    private PointToTransform pointingCommand;
    private ShooterSetVelocityContinuous shootingCommand;
    private HoodSetPositionContinuous hoodCommand;

    private boolean inCommandGroup;

    public LineUpSequence(DriveTrain driveTrain, Indexer indexer, Shooter shooter, Hood hood, ChickenXboxController driverController, LocalizationManager localizationManager, boolean inCommandGroup, boolean useThrottle) {
        this.localizationManager = localizationManager;
        this.inCommandGroup = inCommandGroup;

        this.pointingCommand = new PointToTransform(driveTrain, localizationManager, localizationManager::getSelectedTarget, driverController, false, useThrottle);
        this.shootingCommand = new ShooterSetVelocityContinuous(shooter, localizationManager::getShooterRPMForSelectedGoal);
        this.hoodCommand = new HoodSetPositionContinuous(hood, localizationManager::getHoodTicksForSelectedGoal);

        addCommands(
                parallel(
                        sequence(
                                new InstCommand(localizationManager::selectTarget),
                                new WaitUntilCommand(localizationManager::useLidarForDistanceEst), // TODO: should this really be the trigger?
                                new InstCommand(localizationManager::selectTarget)
                        ),
                        pointingCommand,
                        shootingCommand,
                        hoodCommand
                )
        );
    }

    @Override
    public void execute() {
        SmartDashboard.putNumber("LineUpSequence/getDistanceToSelectedTarget", localizationManager.getDistanceToSelectedTarget());
        SmartDashboard.putBoolean("LineUpSequence/isReadyToShootAll", isLinedUp());
        super.execute();
    }

    public boolean isLinedUp() {
        if (!inCommandGroup) {
            if (!isScheduled()) return false;
        }
        return pointingCommand.hasReachedGoal() && shootingCommand.hasReachedGoal() && hoodCommand.hasReachedGoal() && (localizationManager.isLimelightTargetFound() || localizationManager.getDistanceToSelectedTarget() < 2.3);
    }
}
