package org.team1540.robot2020.commands.shooter;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpiutil.math.MathUtil;
import org.team1540.robot2020.LocalizationManager;
import org.team1540.robot2020.commands.drivetrain.DriveTrain;
import org.team1540.robot2020.commands.drivetrain.PointToTarget;
import org.team1540.robot2020.commands.hood.Hood;
import org.team1540.robot2020.commands.hood.HoodSetPositionContinuous;
import org.team1540.robot2020.utils.ChickenXboxController;
import org.team1540.robot2020.utils.LookupTableUtils;
import org.team1540.rooster.datastructures.threed.Transform3D;
import org.team1540.rooster.datastructures.utils.UnitsUtils;

public class ShooterLineUpSequence extends ParallelCommandGroup {
    private LocalizationManager localization;
    private PointToTarget pointingCommand;
    private ShooterSetVelocityContinuous shootingCommand;
    private HoodSetPositionContinuous hoodCommand;

    private double[] HOOD = new double[]{-224.01043701171875, -139.78330993652344, -105.16877746582031, -95.57478332519531, -77.28832244873047, -71.78778076171875};
    private double[] DISTANCE = new double[]{63.874015748031496, 120.96062992125984, 154.03149606299212, 183.16535433070865, 261.11811023622045, 352.06299212598424};
    private double[] FLYWHEEL = new double[]{1780.0, 2480.0, 2870.0, 5000.0, 5000.0, 5000.0};

    public ShooterLineUpSequence(DriveTrain driveTrain, Shooter shooter, Hood hood, ChickenXboxController driverController, LocalizationManager localization) {
        this.localization = localization;

        this.pointingCommand = new PointToTarget(driveTrain, localization, driverController, false);
        this.shootingCommand = new ShooterSetVelocityContinuous(shooter, () -> {
            double norm = getDistanceInchesOrDefault(localization);
            return MathUtil.clamp(LookupTableUtils.getDoubleLookupTable(norm, DISTANCE, FLYWHEEL), 1000, 5800);
        });
        this.hoodCommand = new HoodSetPositionContinuous(hood, () -> {
            double norm = getDistanceInchesOrDefault(localization);
            return MathUtil.clamp(LookupTableUtils.getDoubleLookupTable(norm, DISTANCE, HOOD), -230, -1);
        });

        addCommands(
                pointingCommand,
                shootingCommand,
                hoodCommand
        );
        resetIndicators();
    }

    private double getDistanceInchesOrDefault(LocalizationManager localization) {
        Transform3D robotToRearHoleTransform = localization.getRobotToRearHoleTransform();
        if (robotToRearHoleTransform == null) return 100; // default cm
        return UnitsUtils.metersToInches(robotToRearHoleTransform.getPosition().getNorm());
    }

    private void resetIndicators() {
        SmartDashboard.putBoolean("ShooterLineUpSequence/isReadyToShootAll", false);
        SmartDashboard.putBoolean("ShooterLineUpSequence/isPointing", false);
        SmartDashboard.putBoolean("ShooterLineUpSequence/isShooterGoal", false);
        SmartDashboard.putBoolean("ShooterLineUpSequence/isHoodGoal", false);
    }

    @Override
    public void execute() {
        SmartDashboard.putNumber("ShooterLineUpSequence/getDistanceInchesOrDefault", getDistanceInchesOrDefault(localization));
        SmartDashboard.putBoolean("ShooterLineUpSequence/isReadyToShootAll", isLinedUp(ll));
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
