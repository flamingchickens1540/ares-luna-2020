package org.team1540.robot2020.utils;

import com.ctre.phoenix.motorcontrol.can.BaseMotorController;

public class CTREBaseMotorControllerEncoder extends Encoder {

    private final BaseMotorController baseMotorController;

    public CTREBaseMotorControllerEncoder(BaseMotorController baseMotorController, double distancePerPulse, boolean inverted) {
        super(distancePerPulse, inverted);
        this.baseMotorController = baseMotorController;
    }

    @Override
    public double getDistanceTicks() {
        return baseMotorController.getSelectedSensorPosition();
    }

    @Override
    public double getRateTicksPerSecond() {
        return (double) baseMotorController.getSelectedSensorVelocity() * 10.0;
    }

    @Override
    public void reset() {
        baseMotorController.setSelectedSensorPosition(0);
    }
}