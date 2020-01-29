package org.team1540.robot2020.subsystems;

import com.ctre.phoenix.motorcontrol.TalonFXControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonFX;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Shooter extends SubsystemBase {
    private TalonFX shooterA = new TalonFX(8);
    private TalonFX shooterB = new TalonFX(9);

    public Shooter() {
        setupMotors();
    }

    private void setupMotors() {
        shooterB.follow(shooterA);
    }

    public void setVelocityTicksPerDecisecond(double velocity) {
        shooterA.set(TalonFXControlMode.Velocity, velocity);
    }

    public void stop() {
        setVelocityTicksPerDecisecond(0);
    }

    public double getVelocityTicksPerDecisecond() {
        return shooterA.getSelectedSensorVelocity();
    }

}
