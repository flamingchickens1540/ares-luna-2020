package org.team1540.robot2020.commands.drivetrain;

import com.kauailabs.navx.frc.ITimestampedDataSubscriber;
import edu.wpi.first.wpilibj.controller.PIDController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import org.team1540.robot2020.utils.ControlUtils;
import org.team1540.robot2020.utils.NavX;

public class SegwayDriveNoCallback extends CommandBase {
    private final PIDController balanceController;
    private NavX navX;
    private DriveTrain driveTrain;
    private double max;
    private double balancePoint = 0;
    private ITimestampedDataSubscriber iTimestampedDataSubscriber;

    public SegwayDriveNoCallback(NavX navX, DriveTrain driveTrain) {
        this.navX = navX;
        this.driveTrain = driveTrain;

        addRequirements(driveTrain);

        SmartDashboard.putNumber("segwayDrive/P", 3.5);
        SmartDashboard.putNumber("segwayDrive/I", 0);
        SmartDashboard.putNumber("segwayDrive/D", 0);
        SmartDashboard.putNumber("segwayDrive/max", 0.1);
        SmartDashboard.putNumber("segwayDrive/balancePointAdjust", 0);

        balanceController = new PIDController(0, 0, 0);
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
        lastEncoderPos = (driveTrain.getDistanceLeft() + driveTrain.getDistanceRight()) / 2;
    }

    double lastEncoderPos = 0;

    @Override
    public void execute() {
        double pitch = navX.getPitch();
        double error = balancePoint - pitch;
        double pidOutput = balanceController.calculate(error);
        double constrainedOuput = ControlUtils.allVelocityConstraints(pidOutput, max, 0, 0);
        SmartDashboard.putNumber("segwayDrive/pitch", pitch);
        SmartDashboard.putNumber("segwayDrive/error", error);
        SmartDashboard.putNumber("segwayDrive/constrainedOuput", constrainedOuput);
        SmartDashboard.putNumber("segwayDrive/balancePoint", balancePoint);
        driveTrain.setPercent(constrainedOuput, constrainedOuput);
        double currentEncoderPos = (driveTrain.getDistanceLeft() + driveTrain.getDistanceRight()) / 2;
        double balancePointAdjust = SmartDashboard.getNumber("segwayDrive/balancePointAdjust", 0);
        double deltaEncoder = currentEncoderPos - lastEncoderPos;
        balancePoint += balancePointAdjust * deltaEncoder;
        SmartDashboard.putNumber("segwayDrive/deltaEncoder", deltaEncoder);
        lastEncoderPos = currentEncoderPos;
    }

    @Override
    public void end(boolean interrupted) {
        driveTrain.setPercent(0, 0);
    }
}
