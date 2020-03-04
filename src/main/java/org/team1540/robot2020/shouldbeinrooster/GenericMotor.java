package org.team1540.robot2020.shouldbeinrooster;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.VictorSPX;
import org.team1540.rooster.wrappers.ChickenTalon;

public class GenericMotor {
    private ChickenTalon talon;
    private VictorSPX victor;
    public int index;

    public GenericMotor(ChickenTalon talon, int index) {
        this.talon = talon;
        this.index = index;
    }

    public GenericMotor(VictorSPX victor, int index) {
        this.victor = victor;
        this.index = index;
    }

    public void setPercent(double percent) {
        if (talon != null) {
            talon.set(ControlMode.PercentOutput, percent);
        } else if (victor != null) {
            victor.set(ControlMode.PercentOutput, percent);
        }
    }
}
