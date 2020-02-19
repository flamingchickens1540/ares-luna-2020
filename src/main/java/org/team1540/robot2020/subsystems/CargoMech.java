package org.team1540.robot2020.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import org.team1540.rooster.wrappers.ChickenTalon;

public class CargoMech extends SubsystemBase {
    public ChickenTalon cargoRollerTop = new ChickenTalon(9);

    public CargoMech() {
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
}
