package org.team1540.robot2020.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import org.team1540.robot2020.shouldbeinrooster.GenericMotor;
import org.team1540.robot2020.shouldbeinrooster.MotorTesting;
import org.team1540.robot2020.shouldbeinrooster.SubsystemBase;
import org.team1540.rooster.wrappers.ChickenTalon;

public class CargoMech extends SubsystemBase {
    public ChickenTalon cargoRollerTop = new ChickenTalon(9);

    public CargoMech() {
        MotorTesting.getInstance().addMotor(new GenericMotor(cargoRollerTop, 6));
    }

    @Override
    public void configMotors() {
        cargoRollerTop.setNeutralMode(NeutralMode.Brake);
    }

    public void setRollerSpeed(double speed) {
        cargoRollerTop.set(ControlMode.PercentOutput, speed);
    }

    @Override
    public void periodic() {
        SmartDashboard.putNumber("cargoMech/encoder", getEncoderticks());
    }

    public double getEncoderticks() {
        return cargoRollerTop.getSelectedSensorPosition();
    }

    public void stop() {
        setRollerSpeed(0);
    }

    public Command commandStop() {
        return new InstantCommand(this::stop, this);
    }
}
