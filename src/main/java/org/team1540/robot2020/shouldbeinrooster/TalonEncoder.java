package org.team1540.robot2020.shouldbeinrooster;

import com.ctre.phoenix.motorcontrol.can.TalonFX;

public class TalonEncoder extends Encoder {

    private final TalonFX talon;

    public TalonEncoder(TalonFX talon) {
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