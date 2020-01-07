package org.team1540.robot2020.shouldbeinrooster;

import com.ctre.phoenix.motorcontrol.can.TalonSRX;

public class TalonEncoder extends Encoder {

    private final TalonSRX talon;

    public TalonEncoder(TalonSRX talon) {
        this.talon = talon;
    }

    @Override
    public double getDistanceTicks() {
        return talon.getSelectedSensorPosition();
    }

    @Override
    public double getRateTicksPerSecond() {
        return talon.getSelectedSensorVelocity() / 10;
    }

    @Override
    public void reset() {
        talon.setSelectedSensorPosition(0);
    }
}