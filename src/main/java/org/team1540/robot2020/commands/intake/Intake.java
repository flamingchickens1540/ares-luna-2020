package org.team1540.robot2020.commands.intake;

import com.revrobotics.*;
import edu.wpi.first.networktables.EntryListenerFlags;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.StartEndCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import org.team1540.robot2020.utils.MotorConfigUtils;

public class Intake extends SubsystemBase {

    private double kP = 1.0E-4;
    private double kD = 0;
    private double kF = 9.2E-5;

    private CANSparkMax rollerMotorA = new CANSparkMax(5, CANSparkMaxLowLevel.MotorType.kBrushless);
    private CANSparkMax rollerMotorB = new CANSparkMax(6, CANSparkMaxLowLevel.MotorType.kBrushless);

    private final CANPIDController pidController = rollerMotorA.getPIDController();
    private CANEncoder rollerEncoder = rollerMotorA.getEncoder();

    public Intake() {
        MotorConfigUtils.setDefaultSparkMaxConfig(rollerMotorA);
        MotorConfigUtils.setDefaultSparkMaxConfig(rollerMotorB);
        rollerMotorA.setSmartCurrentLimit(50);
        rollerMotorA.setSecondaryCurrentLimit(30, 20000);
        rollerMotorB.setSmartCurrentLimit(50);
        rollerMotorB.setSecondaryCurrentLimit(30, 20000);

        rollerMotorB.follow(rollerMotorA, true);

        SmartDashboard.putNumber("intake/tuning/kP", kP);
        SmartDashboard.putNumber("intake/tuning/kD", kD);
        SmartDashboard.putNumber("intake/tuning/kF", kF);

        updatePIDs();
        NetworkTableInstance.getDefault().getTable("SmartDashboard/intake/tuning").addEntryListener((table, key, entry, value, flags) -> updatePIDs(), EntryListenerFlags.kUpdate);
    }

    public void updatePIDs() {
        pidController.setP(SmartDashboard.getNumber("intake/tuning/kP", kP));
        pidController.setD(SmartDashboard.getNumber("intake/tuning/kD", kD));
        pidController.setFF(SmartDashboard.getNumber("intake/tuning/kF", kF));
    }

    @Override
    public void periodic() {
        SmartDashboard.putNumber("intake/rollerVelocity", getVelocity());
        SmartDashboard.putNumber("intake/rollerCurrentA", rollerMotorA.getOutputCurrent());
        SmartDashboard.putNumber("intake/rollerCurrentB", rollerMotorB.getOutputCurrent());
        SmartDashboard.putNumber("intake/rollerTemperatureA", rollerMotorA.getMotorTemperature());
        SmartDashboard.putNumber("intake/rollerTemperatureB", rollerMotorB.getMotorTemperature());
    }

    public void setPercent(double percent) {
        rollerMotorA.set(percent);
    }

    public void setVelocity(double velocity) {
        pidController.setReference(velocity, ControlType.kVelocity);
        SmartDashboard.putNumber("intake/rollerError", getVelocity() - velocity);
    }

    public void stop() {
        setPercent(0);
    }

    public double getVelocity() {
        return rollerEncoder.getVelocity();
    }

    public void setBrake(CANSparkMax.IdleMode mode) {
        rollerMotorA.setIdleMode(mode);
        rollerMotorB.setIdleMode(mode);
    }

    public Command commandStop() {
        return new InstantCommand(this::stop, this);
    }

    public Command commandPercent(double percent) {
        return new StartEndCommand(() -> setPercent(percent), this::stop, this);
    }
}
