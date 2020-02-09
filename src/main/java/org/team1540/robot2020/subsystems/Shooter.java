package org.team1540.robot2020.subsystems;

import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.TalonFXControlMode;
import com.ctre.phoenix.motorcontrol.TalonFXInvertType;
import com.ctre.phoenix.motorcontrol.can.TalonFX;
import com.revrobotics.CANDigitalInput;
import com.revrobotics.CANEncoder;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import org.team1540.robot2020.utils.MotorConfigUtils;

public class Shooter extends SubsystemBase {
    private TalonFX shooterMotorA = new TalonFX(9);
    private TalonFX shooterMotorB = new TalonFX(10);

    private CANSparkMax hoodMotor = new CANSparkMax(11, CANSparkMaxLowLevel.MotorType.kBrushless);
    private CANEncoder hoodEncoder = hoodMotor.getEncoder();
    private CANDigitalInput limitSwitch = hoodMotor.getReverseLimitSwitch(CANDigitalInput.LimitSwitchPolarity.kNormallyOpen);

    public Shooter() {
        setupFlywheelMotors();
        setupHoodMotors();
    }

    private void setupHoodMotors() {
        MotorConfigUtils.setDefaultSparkMaxConfig(hoodMotor);
    }

    private void setupFlywheelMotors() {
        MotorConfigUtils.setDefaultTalonFXConfig(shooterMotorA);
        MotorConfigUtils.setDefaultTalonFXConfig(shooterMotorB);

        shooterMotorA.setNeutralMode(NeutralMode.Coast);
        shooterMotorB.setNeutralMode(NeutralMode.Coast);

        shooterMotorB.follow(shooterMotorA);
        shooterMotorB.setInverted(TalonFXInvertType.OpposeMaster);

        // TODO set the PIDF coefficients plz

        // TODO make a ShooterManualControl so that we can find the F coefficient
    }

    @Override
    public void periodic() {
        SmartDashboard.putNumber("shooter/shooterVelocityA", shooterMotorA.getSelectedSensorVelocity());
        SmartDashboard.putNumber("shooter/shooterVelocityA", shooterMotorB.getSelectedSensorVelocity());
        SmartDashboard.putNumber("shooter/shooterCurrent", shooterMotorA.getStatorCurrent());

        SmartDashboard.putNumber("shooter/hoodPosition", hoodEncoder.getPosition());
        SmartDashboard.putNumber("shooter/hoodCurrent", hoodMotor.getOutputCurrent());
        SmartDashboard.putBoolean("shooter/limitSwitch", limitSwitch.get());
    }

    public void disableMotors() {
        shooterMotorA.set(TalonFXControlMode.PercentOutput, 0);
        hoodMotor.set(0);
    }

    // Flywheel
    public double getFlywheelVelocityTicksPerDecisecond() {
        return shooterMotorA.getSelectedSensorVelocity();
    }

    public void setFlywheelVelocityTicksPerDecisecond(double velocity) {
        shooterMotorA.set(TalonFXControlMode.Velocity, velocity);
    }

    // Hood
    public void setHoodPercent(double speed) {
        hoodMotor.set(speed);
    }

    public double getHoodPosition() {
        return hoodEncoder.getPosition();
    }

    public void setHoodPosition(double position) {
        hoodEncoder.setPosition(position);
    }

    // Hood Limit Switch
    public boolean isLimitSwitchPressed() {
        return limitSwitch.get();
    }

}
