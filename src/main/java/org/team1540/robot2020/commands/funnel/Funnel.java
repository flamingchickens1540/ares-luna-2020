package org.team1540.robot2020.commands.funnel;

import com.revrobotics.CANEncoder;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.StartEndCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import org.team1540.robot2020.utils.MotorConfigUtils;

public class Funnel extends SubsystemBase {

    private CANSparkMax funnelLeftMotor = new CANSparkMax(12, CANSparkMaxLowLevel.MotorType.kBrushless);
    private CANSparkMax funnelRightMotor = new CANSparkMax(7, CANSparkMaxLowLevel.MotorType.kBrushless);
    private CANEncoder funnelLeftEncoder = funnelLeftMotor.getEncoder();
    private CANEncoder funnelRightEncoder = funnelRightMotor.getEncoder();

    public Funnel() {
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
//        funnelRightMotor.set(percentRight);
    }

    public void setPercent(double percent) {
        setPercent(percent, percent);
    }

    public void stop() {
        setPercent(0, 0);
    }

    public Command commandStop() {
        return new InstantCommand(this::stop, this);
    }

    public Command commandPercent(double left, double right) {
        return new StartEndCommand(() -> setPercent(left, right), this::stop, this);
    }
}
