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
        // TODO this should restoreFactoryDefaults() on all motors

        // TODO set brake mode on all motors to coast
        // TODO figure out current limit
        shooterB.follow(shooterA);
    }

    public void setVelocityTicksPerDecisecond(double velocity) {
        shooterA.set(TalonFXControlMode.Velocity, velocity);
    }

    public void stop() {
        shooterA.set(TalonFXControlMode.PercentOutput, 0);
    }

    public double getVelocityTicksPerDecisecond() {
        return shooterA.getSelectedSensorVelocity();
    }

}
