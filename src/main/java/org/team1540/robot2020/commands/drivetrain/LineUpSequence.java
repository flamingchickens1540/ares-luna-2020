package org.team1540.robot2020.commands.drivetrain;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;
import edu.wpi.first.wpiutil.math.MathUtil;
import org.apache.commons.math3.geometry.euclidean.threed.Vector3D;
import org.apache.commons.math3.geometry.euclidean.twod.Vector2D;
import org.team1540.robot2020.LocalizationManager;
import org.team1540.robot2020.commands.hood.Hood;
import org.team1540.robot2020.commands.hood.HoodSetPositionContinuous;
import org.team1540.robot2020.commands.indexer.Indexer;
import org.team1540.robot2020.commands.shooter.Shooter;
import org.team1540.robot2020.commands.shooter.ShooterSetVelocityContinuous;
import org.team1540.robot2020.utils.ChickenXboxController;
import org.team1540.robot2020.utils.InstCommand;
import org.team1540.robot2020.utils.LookupTableUtils;
import org.team1540.rooster.datastructures.threed.Transform3D;

public class LineUpSequence extends SequentialCommandGroup {
    private LocalizationManager localization;
    private PointToTransform pointingCommand;
    private ShooterSetVelocityContinuous shootingCommand;
    private HoodSetPositionContinuous hoodCommand;
    private boolean selectedInnerPort = false;

    private double[] DISTANCE = new double[]{1.7430, 1.8630, 1.9018, 2.3105, 3.1180, 4.0000, 4.5735, 6.0000, 7.1455, 8.11, 9.60, 11.96};
    private double[] HOOD = new double[]{-284.7406, -262.6959, -235.6042, -194.8952, -138.9739, -112.2393, -111.2870, -94.1226, -79.7886, -89.22, -92.74, -110.53};
    private double[] FLYWHEEL = new double[]{1537.4147, 1672.9528, 1643.7841, 1882.7768, 2575.7416, 3000.0000, 4142.5895, 4598.9504, 5208.7773, 5417.347714, 5417.347714, 5417.347714};
    private boolean inCommandGroup;

    public LineUpSequence(DriveTrain driveTrain, Indexer indexer, Shooter shooter, Hood hood, ChickenXboxController driverController, LocalizationManager localizationManager, boolean inCommandGroup, boolean useThrottle) {
        this.localization = localizationManager;
        this.inCommandGroup = inCommandGroup;

        this.pointingCommand = new PointToTransform(driveTrain, localizationManager, () -> getSelectedTarget(localizationManager), driverController, false, useThrottle);
        this.shootingCommand = new ShooterSetVelocityContinuous(shooter, () -> {
            double norm = getDistanceMetersOrDefault(localizationManager);
            return MathUtil.clamp(LookupTableUtils.getDoubleLookupTable(norm, DISTANCE, FLYWHEEL), 500, 5800);
        });
        this.hoodCommand = new HoodSetPositionContinuous(hood, () -> {
            double norm = getDistanceMetersOrDefault(localizationManager) - hood.offset;
            return MathUtil.clamp(LookupTableUtils.getDoubleLookupTable(norm, DISTANCE, HOOD), -294, -1);
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

    @Override
    public void initialize() {
        localization.forceLimelightLedsOn(true);
        super.initialize();
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
        SmartDashboard.putBoolean("LineUpSequence/isReadyToShootAll", false);
        SmartDashboard.putBoolean("LineUpSequence/isPointing", false);
        SmartDashboard.putBoolean("LineUpSequence/isShooterGoal", false);
        SmartDashboard.putBoolean("LineUpSequence/isHoodGoal", false);
    }

    @Override
    public void execute() {
        SmartDashboard.putNumber("LineUpSequence/getDistanceMetersOrDefault", getDistanceMetersOrDefault(localization));
        SmartDashboard.putBoolean("LineUpSequence/isReadyToShootAll", isLinedUp());
        SmartDashboard.putBoolean("LineUpSequence/isPointing", pointingCommand.hasReachedGoal());
        SmartDashboard.putBoolean("LineUpSequence/isShooterGoal", shootingCommand.hasReachedGoal());
        SmartDashboard.putBoolean("LineUpSequence/isHoodGoal", hoodCommand.hasReachedGoal());
        SmartDashboard.putBoolean("LineUpSequence/isLimelightTargetFound", localization.isLimelightTargetFound());

        super.execute();
    }

    public boolean isLinedUp() {
        if (!inCommandGroup) {
            if (!isScheduled()) return false;
        }
        return pointingCommand.hasReachedGoal() && shootingCommand.hasReachedGoal() && hoodCommand.hasReachedGoal() && (localization.isLimelightTargetFound() || getDistanceMetersOrDefault(localization) < 2.3);
    }

    @Override
    public void end(boolean interrupted) {
        super.end(interrupted);
        resetIndicators();
        localization.forceLimelightLedsOn(false);
    }
}
