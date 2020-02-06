package org.team1540.robot2020.subsystems;

import com.revrobotics.CANEncoder;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Intake extends SubsystemBase {

    // todo Should intake rollers and funnel be separate subsystems?
    private CANSparkMax rollerMotor = new CANSparkMax(4, CANSparkMaxLowLevel.MotorType.kBrushless);
    private CANEncoder rollerEncoder = rollerMotor.getEncoder();

    private CANSparkMax funnelLeftMotor = new CANSparkMax(5, CANSparkMaxLowLevel.MotorType.kBrushless);
    private CANSparkMax funnelRightMotor = new CANSparkMax(6, CANSparkMaxLowLevel.MotorType.kBrushless);
    private CANEncoder funnelLeftEncoder = funnelLeftMotor.getEncoder();
    private CANEncoder funnelRightEncoder = funnelRightMotor.getEncoder();

    public static final double defaultRollerPercent = 1;
    public static final double defaultFunnelLeftPercent = 1;
    public static final double defaultFunnelRightPercent = 1;

    public Intake() {
        // TODO figure out brake mode on all motors
        // TODO figure out current limit on all motors
        rollerMotor.restoreFactoryDefaults();
        funnelLeftMotor.restoreFactoryDefaults();
        funnelRightMotor.restoreFactoryDefaults();

        rollerMotor.setSmartCurrentLimit(20);
        funnelLeftMotor.setSmartCurrentLimit(20);
        funnelRightMotor.setSmartCurrentLimit(20);
    }

    @Override
    public void periodic() {
        SmartDashboard.putNumber("intake/rollerVelocity", rollerEncoder.getVelocity());
        SmartDashboard.putNumber("intake/funnelLeftVelocity", funnelLeftEncoder.getVelocity());
        SmartDashboard.putNumber("intake/funnelRightVelocity", funnelRightEncoder.getVelocity());
    }

    public void setRollerPercent(double percent) {
        rollerMotor.set(percent);
    }

    public void setFunnelPercent(double percentLeft, double percentRight) {
        funnelLeftMotor.set(percentLeft);
        funnelRightMotor.set(percentRight);
    }

    public void setFunnelAndRollerPercent(double rollerPercent, int percentLeft, int percentRight) {
        setRollerPercent(rollerPercent);
        setFunnelPercent(percentLeft, percentRight);
    }

    public void setFunnelAndRollerPercent(boolean intake) {
        int inversion = intake ? 1 : -1;
        setRollerPercent(inversion * Intake.defaultRollerPercent);
        setFunnelPercent(inversion * Intake.defaultFunnelLeftPercent, inversion * Intake.defaultFunnelRightPercent);
    }

    public void stop() {
        setFunnelAndRollerPercent(true);
    }
}
