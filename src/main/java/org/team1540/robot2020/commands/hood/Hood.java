package org.team1540.robot2020.commands.hood;

import com.revrobotics.*;
import edu.wpi.first.networktables.EntryListenerFlags;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import org.team1540.robot2020.utils.MotorConfigUtils;

public class Hood extends SubsystemBase {
    private final double kP = 2;
    private final double kD = 0;

    private CANSparkMax hoodMotor = new CANSparkMax(11, CANSparkMaxLowLevel.MotorType.kBrushless);
    CANPIDController hoodController = new CANPIDController(hoodMotor);
    private CANEncoder hoodEncoder = hoodMotor.getEncoder();
    private CANDigitalInput limitSwitch = hoodMotor.getForwardLimitSwitch(CANDigitalInput.LimitSwitchPolarity.kNormallyOpen);

    public Hood() {
        MotorConfigUtils.setDefaultSparkMaxConfig(hoodMotor);

        SmartDashboard.putNumber("hood/tuning/kP", kP);
        SmartDashboard.putNumber("hood/tuning/kD", kD);

        updatePIDs();

        NetworkTableInstance.getDefault().getTable("SmartDashboard/hood/tuning").addEntryListener((table, key, entry, value, flags) -> updatePIDs(), EntryListenerFlags.kUpdate);
    }

    @Override
    public void periodic() {
        SmartDashboard.putNumber("hood/position", hoodEncoder.getPosition());
        SmartDashboard.putNumber("hood/current", hoodMotor.getOutputCurrent());
        SmartDashboard.putBoolean("hood/limitSwitch", limitSwitch.get());
        // TODO is this the value we want to be putting up
        SmartDashboard.putNumber("hood/error", hoodController.getSmartMotionAllowedClosedLoopError(0));

    }

    public void disableMotors() {
        hoodMotor.set(0);
    }

    public void updatePIDs() {
        hoodController.setP(SmartDashboard.getNumber("hood/tuning/kP", kP));
        hoodController.setD(SmartDashboard.getNumber("hood/tuning/kD", kD));
    }

    public double getPosition() {
        return hoodEncoder.getPosition();
    }

    public void setPosition(double position) {
        hoodController.setReference(position, ControlType.kPosition);
    }

    public void setPercent(double speed) {
        hoodMotor.set(speed);
    }

    public void zero() {
        hoodEncoder.setPosition(0);
    }

    public boolean isLimitSwitchPressed() {
        return limitSwitch.get();
    }
}
