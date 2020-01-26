package org.team1540.robot2020.utils;

import com.ctre.phoenix.motorcontrol.can.BaseMotorController;

public class CTREMotorControllerEncoder extends Encoder {

    private final BaseMotorController talon;

    public CTREMotorControllerEncoder(BaseMotorController talon) {
        this.talon = talon;
    }

    @Override
    public double getDistanceTicks() {
        return talon.getSelectedSensorPosition();
    }

    @Override
    public double getRateTicksPerSecond() {
        return (double) talon.getSelectedSensorVelocity() * 10.0;
    }

    @Override
    public void reset() {
        talon.setSelectedSensorPosition(0);
    }
}