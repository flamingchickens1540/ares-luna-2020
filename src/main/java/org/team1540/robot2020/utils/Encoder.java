package org.team1540.robot2020.utils;

public abstract class Encoder {

    private double distancePerPulse = 1;
    private double inversionNumber = 1;

    public Encoder(double distancePerPulse, boolean inverted) {
        this.distancePerPulse = distancePerPulse;
        this.inversionNumber = inverted ? -1 : 1;
    }

    public double getDistance() {
        return inversionNumber * distancePerPulse * getDistanceTicks();
    }

    public double getRate() {
        return inversionNumber * distancePerPulse * getRateTicksPerSecond();
    }

    public abstract double getDistanceTicks();

    public abstract double getRateTicksPerSecond();

    public abstract void reset();
}
