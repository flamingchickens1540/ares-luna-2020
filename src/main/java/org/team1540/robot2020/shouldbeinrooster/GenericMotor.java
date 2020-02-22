package org.team1540.robot2020.shouldbeinrooster;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.VictorSPX;
import org.team1540.rooster.wrappers.ChickenTalon;

public class GenericMotor {
    private ChickenTalon talon;
    private VictorSPX victor;

    public GenericMotor(ChickenTalon talon) {
        this.talon = talon;
    }

    public GenericMotor(VictorSPX victor) {
        this.victor = victor;
    }

    public void setPercent(double percent) {
        if (talon != null) {
            talon.set(ControlMode.PercentOutput, percent);
        } else {
            victor.set(ControlMode.PercentOutput, percent);
        }
    }
}
