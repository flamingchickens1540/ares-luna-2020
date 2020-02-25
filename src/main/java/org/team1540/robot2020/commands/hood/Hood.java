package org.team1540.robot2020.commands.hood;

import com.revrobotics.*;
import edu.wpi.first.networktables.EntryListenerFlags;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import org.team1540.robot2020.utils.MotorConfigUtils;

public class Hood extends SubsystemBase {
    private final double kP = 0.5;
    private final double kD = 0;

    public double offset = 0;

    private CANSparkMax hoodMotor = new CANSparkMax(11, CANSparkMaxLowLevel.MotorType.kBrushless);
    private CANPIDController hoodController = new CANPIDController(hoodMotor);
    private CANEncoder hoodEncoder = hoodMotor.getEncoder();
    private CANDigitalInput limitSwitch = hoodMotor.getForwardLimitSwitch(CANDigitalInput.LimitSwitchPolarity.kNormallyOpen);
    private double lastSetpoint;

    public Hood() {
        MotorConfigUtils.setDefaultSparkMaxConfig(hoodMotor);
        hoodMotor.setIdleMode(CANSparkMax.IdleMode.kBrake);

        SmartDashboard.putNumber("hood/tuning/kP", kP);
        SmartDashboard.putNumber("hood/tuning/kD", kD);

        updatePIDs();

        NetworkTableInstance.getDefault().getTable("SmartDashboard/hood/tuning").addEntryListener((table, key, entry, value, flags) -> updatePIDs(), EntryListenerFlags.kUpdate);
    }

    @Override
    public void periodic() {
        SmartDashboard.putNumber("hood/lastSetpoint", hoodEncoder.getPosition());
        SmartDashboard.putNumber("hood/current", hoodMotor.getOutputCurrent());
        SmartDashboard.putBoolean("hood/limitSwitch", limitSwitch.get());
        SmartDashboard.putNumber("hood/error", hoodEncoder.getPosition() - lastSetpoint); // https://trello.com/c/xymF8avF/84-get-closed-loop-error-function
    }

    public void stop() {
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
        this.lastSetpoint = position;
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

    public Command commandStop() {
        return new InstantCommand(this::stop, this);
    }
}
