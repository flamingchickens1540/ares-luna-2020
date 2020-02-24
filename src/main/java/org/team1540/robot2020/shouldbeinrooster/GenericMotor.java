package org.team1540.robot2020.shouldbeinrooster;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.VictorSPX;
import org.team1540.rooster.wrappers.ChickenTalon;

public class GenericMotor {
    private ChickenTalon talon;
    private VictorSPX victor;
    public int index;
    public String name;

    public GenericMotor() {}

    public GenericMotor(ChickenTalon talon, int index, String name) {
        this.talon = talon;
        this.index = index;
        this.name = name;
    }

    public GenericMotor(VictorSPX victor, int index, String name) {
        this.victor = victor;
        this.index = index;
        this.name = name;
    }

    public void setPercent(double percent) {
        if (talon != null) {
            talon.set(ControlMode.PercentOutput, percent);
        } else if (victor != null) {
            victor.set(ControlMode.PercentOutput, percent);
        }
    }
}
