package org.team1540.robot2020.commands.drivetrain;

import com.kauailabs.navx.frc.ITimestampedDataSubscriber;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import org.team1540.robot2020.utils.ControlUtils;
import org.team1540.robot2020.utils.MiniPID;
import org.team1540.robot2020.utils.NavX;

public class SegwayDrive extends CommandBase {
    private final MiniPID balanceController;
    private NavX navX;
    private DriveTrain driveTrain;
    private double max;
    private double balancePoint = 0;
    private ITimestampedDataSubscriber iTimestampedDataSubscriber;

    public SegwayDrive(NavX navX, DriveTrain driveTrain) {
        this.navX = navX;
        this.driveTrain = driveTrain;

        addRequirements(driveTrain);

        SmartDashboard.putNumber("segwayDrive/P", 0.3);
        SmartDashboard.putNumber("segwayDrive/I", 0);
        SmartDashboard.putNumber("segwayDrive/D", 0);
        SmartDashboard.putNumber("segwayDrive/max", 0.45);

        balanceController = new MiniPID(0, 0, 0);
    }

    public void setBalancePoint() {
        balancePoint = navX.getPitch();
    }

    @Override
    public void initialize() {
        double p = SmartDashboard.getNumber("segwayDrive/P", 0);
        double i = SmartDashboard.getNumber("segwayDrive/I", 0);
        double d = SmartDashboard.getNumber("segwayDrive/D", 0);
        max = SmartDashboard.getNumber("segwayDrive/max", 0);
        balanceController.setPID(p, i, d);
        iTimestampedDataSubscriber = (system_timestamp, sensor_timestamp, sensor_data, context) -> {
            double pitch = NavX.radiansFromNavXRaw(sensor_data.pitch);
            double error = balancePoint - pitch;
            double pidOutput = balanceController.getOutput(error);
            double constrainedOuput = ControlUtils.allVelocityConstraints(pidOutput, max, 0, 0);
            SmartDashboard.putNumber("segwayDrive/pitch", pitch);
            SmartDashboard.putNumber("segwayDrive/error", error);
            SmartDashboard.putNumber("segwayDrive/constrainedOuput", constrainedOuput);
            driveTrain.setPercent(constrainedOuput, constrainedOuput);
        };
        navX.registerCallback(iTimestampedDataSubscriber, this);
    }

    @Override
    public void end(boolean interrupted) {
        navX.deregisterCallback(iTimestampedDataSubscriber);
        driveTrain.setPercent(0, 0);
    }
}
