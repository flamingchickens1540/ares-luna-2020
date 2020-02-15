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
import org.team1540.robot2020.utils.NavX;
import org.team1540.rooster.wrappers.Limelight;

public class ShooterLineUpSequence extends ParallelCommandGroup {
    private LocalizationManager localization;
    private PointToTarget pointingCommand;
    private ShooterSetVelocityContinuous shootingCommand;
    private HoodSetPositionContinuous hoodCommand;

    private double[] HOOD = new double[]{-71.78778076171875, -77.28832244873047, -95.57478332519531, -105.16877746582031, -139.78330993652344, -224.01043701171875};
    private double[] DISTANCE = new double[]{352.06299212598424, 261.11811023622045, 183.16535433070865, 154.03149606299212, 120.96062992125984, 63.874015748031496};
    private double[] FLYWHEEL = new double[]{5000.0, 5000.0, 5000.0, 2870.0, 2480.0, 1780.0};

    private double lastLidarDistance = 200;

    public ShooterLineUpSequence(NavX navx, DriveTrain driveTrain, ChickenXboxController driverController, Limelight limelight, Shooter shooter, Hood hood, LocalizationManager localization) {
        this.localization = localization;

        // TODO: fix D constant for flywheel
        this.pointingCommand = new PointToTarget(navx, driveTrain, driverController, limelight);
        this.shootingCommand = new ShooterSetVelocityContinuous(shooter, () -> MathUtil.clamp(getDoubleLookupTable(lastLidarDistance, DISTANCE, FLYWHEEL), 1000, 5800));
        this.hoodCommand = new HoodSetPositionContinuous(hood, () -> MathUtil.clamp(getDoubleLookupTable(lastLidarDistance, DISTANCE, HOOD), -230, -1));

        addCommands(
                pointingCommand,
                shootingCommand,
                hoodCommand
        );
    }

    @Override
    public void execute() {
        super.execute();

        if (pointingCommand.hasReachedGoal()) {
            lastLidarDistance = localization.getLidar().getDistance();
        }

        SmartDashboard.putNumber("ShooterLineUpSequence/lastLidarDistance", lastLidarDistance);
    }

    private static double line(double x1, double y1, double x2, double y2, double x) {
        return ((y2 - y1) / (x2 - x1)) * (x - x1) + y1;
    }

    /**
     * @param input x-value to calculate
     * @param xarr  sorted (smallest to largest) array of x-values
     * @param yarr  sorted (smallest to largest) array of y-values
     * @return the best y-value for the lookup table
     * TODO simplify this logic
     */
    private static double getDoubleLookupTable(double input, double[] xarr, double[] yarr) {
        if (input < xarr[0]) { // If distance less than smallest recorded distance
            double lowerX = xarr[0];
            double lowerY = yarr[0];

            double upperX = xarr[1];
            double upperY = yarr[1];

            return line(lowerX, lowerY, upperX, upperY, input);
        } else if (input > xarr[xarr.length - 1]) { // If distance greater than largest recorded value
            double lowerX = xarr[xarr.length - 2];
            double lowerY = yarr[yarr.length - 2];

            double upperX = xarr[xarr.length - 1];
            double upperY = yarr[yarr.length - 1];

            return line(lowerX, lowerY, upperX, upperY, input);
        } else { // If distance somewhere in the middle
            for (int i = 0; i < xarr.length - 1; i++) {
                double lowerX = xarr[i];
                double lowerY = yarr[i];

                double upperX = xarr[i + 1];
                double upperY = yarr[i + 1];

                if (lowerX < input) { // HOOD[i-1] < distance < HOOD[i+1]
                    return line(lowerX, lowerY, upperX, upperY, input);
                }
            }
        }
        return 0;
    }

    public boolean isLinedUp() {
        if (!isScheduled()) return false;
        return pointingCommand.hasReachedGoal() && shootingCommand.hasReachedGoal() && hoodCommand.hasReachedGoal();
    }

}
