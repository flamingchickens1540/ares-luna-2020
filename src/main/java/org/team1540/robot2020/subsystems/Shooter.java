package org.team1540.robot2020.subsystems;

import com.ctre.phoenix.motorcontrol.TalonFXControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonFX;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Shooter extends SubsystemBase {
    private TalonFX shooterA = new TalonFX(0);
    private TalonFX shooterB = new TalonFX(0);

    public Shooter() {
        shooterB.follow(shooterA);
    }

    public void setPercent(double speed) {
        shooterA.set(TalonFXControlMode.PercentOutput, speed);
    }

    public void setVelocity(double velocity) {
        shooterA.set(TalonFXControlMode.Velocity, velocity);
    }

    public double getVelocity() {
        return shooterA.getSelectedSensorVelocity();
    }
}
