package org.team1540.robot2020.subsystems;

import com.revrobotics.*;
import edu.wpi.first.networktables.EntryListenerFlags;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import org.team1540.robot2020.utils.MotorConfigUtils;

public class Intake extends SubsystemBase {

    private double kP = 2.0E-4;
    private double kD = 100.0;
    private double kF = 9.2E-5;

    private CANSparkMax rollerMotor = new CANSparkMax(5, CANSparkMaxLowLevel.MotorType.kBrushless);
    private final CANPIDController pidController = rollerMotor.getPIDController();
    private CANEncoder rollerEncoder = rollerMotor.getEncoder();

    public static final double defaultRollerPercent = 1;

    public Intake() {
        // TODO figure out brake mode on all motors
        // TODO figure out current limit on all motors
        MotorConfigUtils.setDefaultSparkMaxConfig(rollerMotor);
        rollerMotor.setSmartCurrentLimit(50);
        rollerMotor.setSecondaryCurrentLimit(20, 20000);

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
        SmartDashboard.putNumber("intake/rollerVelocity", rollerEncoder.getVelocity());
        SmartDashboard.putNumber("intake/rollerCurrent", rollerMotor.getOutputCurrent());
        SmartDashboard.putNumber("intake/rollerTemperature", rollerMotor.getMotorTemperature());
    }

    public void setVelocity(double velocity) {
        pidController.setReference(velocity, ControlType.kVelocity);
    }

    public void setPercent(double percent) {
        rollerMotor.set(percent);
    }

    public void stop() {
        setPercent(0);
    }

    public double getVelocity() {
        return rollerEncoder.getVelocity();
    }
}
