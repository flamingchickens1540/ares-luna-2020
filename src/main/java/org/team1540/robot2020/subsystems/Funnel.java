package org.team1540.robot2020.subsystems;

import com.revrobotics.CANEncoder;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import org.team1540.robot2020.utils.MotorConfigUtils;

public class Funnel extends SubsystemBase {

    private CANSparkMax funnelLeftMotor = new CANSparkMax(6, CANSparkMaxLowLevel.MotorType.kBrushless);
    private CANSparkMax funnelRightMotor = new CANSparkMax(7, CANSparkMaxLowLevel.MotorType.kBrushless);
    private CANEncoder funnelLeftEncoder = funnelLeftMotor.getEncoder();
    private CANEncoder funnelRightEncoder = funnelRightMotor.getEncoder();

    public static final double defaultFunnelLeftPercent = 1;
    public static final double defaultFunnelRightPercent = 1;

    public Funnel() {
        // TODO figure out brake mode on all motors
        // TODO figure out current limit on all motors
        MotorConfigUtils.setDefaultSparkMaxConfig(funnelLeftMotor);
        MotorConfigUtils.setDefaultSparkMaxConfig(funnelRightMotor);
        funnelLeftMotor.setInverted(true);

    }

    @Override
    public void periodic() {
        SmartDashboard.putNumber("intake/funnelLeftVelocity", funnelLeftEncoder.getVelocity());
        SmartDashboard.putNumber("intake/funnelRightVelocity", funnelRightEncoder.getVelocity());
        SmartDashboard.putNumber("intake/funnelLeftCurrent", funnelLeftMotor.getOutputCurrent());
        SmartDashboard.putNumber("intake/funnelRightCurrent", funnelRightMotor.getOutputCurrent());
    }

    public void setPercent(double percentLeft, double percentRight) {
        funnelLeftMotor.set(percentLeft);
        funnelRightMotor.set(percentRight);
    }

    public void setPercent(double percent) {
        setPercent(percent, percent);
    }

    public void stop() {
        setPercent(0, 0);
    }
}
