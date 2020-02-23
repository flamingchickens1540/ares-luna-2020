package org.team1540.robot2020.commands.shooter;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;
import edu.wpi.first.wpiutil.math.MathUtil;
import org.apache.commons.math3.geometry.euclidean.threed.Vector3D;
import org.apache.commons.math3.geometry.euclidean.twod.Vector2D;
import org.team1540.robot2020.LocalizationManager;
import org.team1540.robot2020.commands.drivetrain.DriveTrain;
import org.team1540.robot2020.commands.drivetrain.PointToTarget;
import org.team1540.robot2020.commands.hood.Hood;
import org.team1540.robot2020.commands.hood.HoodSetPositionContinuous;
import org.team1540.robot2020.commands.indexer.Indexer;
import org.team1540.robot2020.utils.ChickenXboxController;
import org.team1540.robot2020.utils.InstCommand;
import org.team1540.robot2020.utils.LookupTableUtils;
import org.team1540.rooster.datastructures.threed.Transform3D;

public class ShooterLineUpSequence extends SequentialCommandGroup {
    private LocalizationManager localization;
    private PointToTarget pointingCommand;
    private ShooterSetVelocityContinuous shootingCommand;
    private HoodSetPositionContinuous hoodCommand;
    private boolean selectedInnerPort = false;

    private double[] DISTANCE = new double[]{1.901766083, 2.310501969, 3.137942909, 4.573503095, 6.597244189, 7.3940, 7.394777626, 8.012866039, 8.780491971, 10.71451055};
    private double[] HOOD = new double[]{-235.6041718, -194.8951874, -177.2308197, -134.8553925, -111.3584442, -112.6201859, -78.07411194, -72.45451355, -63.16789246, -59.95329285};
    private double[] FLYWHEEL = new double[]{1643.784083, 1882.776796, 1988.624347, 2498.950448, 2982.554941, 3014.254268, 4629.654212, 4608.690124, 5493.781097, 5643.453423};

    public ShooterLineUpSequence(DriveTrain driveTrain, Indexer indexer, Shooter shooter, Hood hood, ChickenXboxController driverController, LocalizationManager localizationManager) {
        this.localization = localizationManager;

        this.pointingCommand = new PointToTarget(driveTrain, localizationManager, () -> getSelectedTarget(localizationManager), driverController, false);
        this.shootingCommand = new ShooterSetVelocityContinuous(shooter, () -> {
            double norm = getDistanceMetersOrDefault(localizationManager);
            return MathUtil.clamp(LookupTableUtils.getDoubleLookupTable(norm, DISTANCE, FLYWHEEL), 1000, 5800);
        });
        this.hoodCommand = new HoodSetPositionContinuous(hood, () -> {
            double norm = getDistanceMetersOrDefault(localizationManager);
            return MathUtil.clamp(LookupTableUtils.getDoubleLookupTable(norm, DISTANCE, HOOD), -230, -1);
        });

        addCommands(
                new InstCommand(localizationManager::startAcceptingLimelight),
                parallel(
                        sequence(
                                new InstCommand(() -> selectedInnerPort = localizationManager.shouldTargetInnerPort()),
                                new WaitUntilCommand(localizationManager::useLidarForDistanceEst), // TODO: should this really be the trigger?
                                new InstCommand(() -> selectedInnerPort = localizationManager.shouldTargetInnerPort())
                        ),
                        pointingCommand,
                        shootingCommand,
                        hoodCommand
                )
        );
        resetIndicators();
    }

    private double getDistanceMetersOrDefault(LocalizationManager localization) {
        Transform3D robotToRearHoleTransform = getSelectedTarget(localization);
        if (robotToRearHoleTransform == null) return 4; // default m
        Vector3D position = robotToRearHoleTransform.getPosition();
        Vector2D pos2d = new Vector2D(position.getX(), position.getY());
        return pos2d.getNorm() + (selectedInnerPort ? 0 : 0.5);
    }

    private Transform3D getSelectedTarget(LocalizationManager localization) {
        return selectedInnerPort ? localization.getRobotToRearHoleTransform() : localization.getRobotToHexagonTransform();
    }

    private void resetIndicators() {
        SmartDashboard.putBoolean("ShooterLineUpSequence/isReadyToShootAll", false);
        SmartDashboard.putBoolean("ShooterLineUpSequence/isPointing", false);
        SmartDashboard.putBoolean("ShooterLineUpSequence/isShooterGoal", false);
        SmartDashboard.putBoolean("ShooterLineUpSequence/isHoodGoal", false);
    }

    @Override
    public void execute() {
        SmartDashboard.putNumber("ShooterLineUpSequence/getDistanceMetersOrDefault", getDistanceMetersOrDefault(localization));
        SmartDashboard.putBoolean("ShooterLineUpSequence/isReadyToShootAll", isLinedUp());
        SmartDashboard.putBoolean("ShooterLineUpSequence/isPointing", pointingCommand.hasReachedGoal());
        SmartDashboard.putBoolean("ShooterLineUpSequence/isShooterGoal", shootingCommand.hasReachedGoal());
        SmartDashboard.putBoolean("ShooterLineUpSequence/isHoodGoal", hoodCommand.hasReachedGoal());

        super.execute();
    }

    public boolean isLinedUp() {
        if (!isScheduled()) return false;
        return pointingCommand.hasReachedGoal() && shootingCommand.hasReachedGoal() && hoodCommand.hasReachedGoal();
    }

    @Override
    public void end(boolean interrupted) {
        super.end(interrupted);
        resetIndicators();
    }
}
