package org.team1540.robot2020.subsystems;

import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.TalonFXControlMode;
import com.ctre.phoenix.motorcontrol.can.BaseMotorController;
import com.ctre.phoenix.motorcontrol.can.TalonFX;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Shooter extends SubsystemBase {
    private TalonFX shooterA = new TalonFX(7);
    private TalonFX shooterB = new TalonFX(8);

    public Shooter() {
        setupMotors();
    }

    private void setupMotors() {
        shooterB.follow(shooterA);
    }

    public void setVelocityTicksPerDecisecond(double velocity) {
        shooterA.set(TalonFXControlMode.Velocity, velocity);
    }

    public double getVelocityTicksPerDecisecond() {
        return shooterA.getSelectedSensorVelocity();
    }
}
