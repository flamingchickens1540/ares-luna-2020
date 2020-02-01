package org.team1540.robot2020.subsystems;

import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.TalonFXControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonFX;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Shooter extends SubsystemBase {
    private TalonFX shooterMotorA = new TalonFX(8);
    private TalonFX shooterMotorB = new TalonFX(9);

    public Shooter() {
        setupMotors();
    }

    private void setupMotors() {
        // TODO figure out current limit
        shooterMotorA.configFactoryDefault();
        shooterMotorB.configFactoryDefault();

        shooterMotorA.setNeutralMode(NeutralMode.Coast);
        shooterMotorB.setNeutralMode(NeutralMode.Coast);

        shooterMotorB.follow(shooterMotorA);
    }

    public void setVelocityTicksPerDecisecond(double velocity) {
        shooterMotorA.set(TalonFXControlMode.Velocity, velocity);
    }

    public void stop() {
        shooterMotorA.set(TalonFXControlMode.PercentOutput, 0);
    }

    public double getVelocityTicksPerDecisecond() {
        return shooterMotorA.getSelectedSensorVelocity();
    }

}
