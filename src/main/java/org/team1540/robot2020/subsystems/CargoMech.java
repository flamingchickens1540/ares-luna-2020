package org.team1540.robot2020.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import org.team1540.rooster.wrappers.ChickenTalon;

public class CargoMech extends SubsystemBase {
    private ChickenTalon cargoRollerTop = new ChickenTalon(9);

    public void setRollerSpeed(double speed) {
        cargoRollerTop.set(ControlMode.PercentOutput, speed);
    }
}
