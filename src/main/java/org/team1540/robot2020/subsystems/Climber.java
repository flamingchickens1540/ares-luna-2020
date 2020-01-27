package org.team1540.robot2020.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonFX;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Climber extends SubsystemBase {
    private TalonFX climberA = new TalonFX(12);
    private TalonFX climberB = new TalonFX(13);

    public Climber() {
        climberB.follow(climberA);
    }

    public void setPercent(double percent) {
        climberA.set(ControlMode.PercentOutput, percent);
    }

    public double getPosition() {
        return climberA.getSelectedSensorPosition();
    }
}
