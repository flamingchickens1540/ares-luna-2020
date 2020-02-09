package org.team1540.robot2020.subsystems;

import com.revrobotics.CANEncoder;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import org.team1540.robot2020.utils.MotorConfigUtils;

public class Intake extends SubsystemBase {

    // todo Should intake rollers and funnel be separate subsystems?
    private CANSparkMax rollerMotor = new CANSparkMax(5, CANSparkMaxLowLevel.MotorType.kBrushless);
    private CANEncoder rollerEncoder = rollerMotor.getEncoder();

    public static final double defaultRollerPercent = 1;

    public Intake() {
        // TODO figure out brake mode on all motors
        // TODO figure out current limit on all motors
        MotorConfigUtils.setDefaultSparkMaxConfig(rollerMotor);
    }

    @Override
    public void periodic() {
        SmartDashboard.putNumber("intake/rollerVelocity", rollerEncoder.getVelocity());
    }

    public void setPercent(double percent) {
        rollerMotor.set(percent);
    }

    public void stop() {
        setPercent(0);
    }
}
