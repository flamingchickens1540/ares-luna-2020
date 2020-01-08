package org.team1540.robot2020.shouldbeinrooster;

public abstract class Encoder {

    private double distancePerPulse = 1;
    private double inversionNumber = 1;

    public void setInverted(boolean inverted) {
        this.inversionNumber = inverted ? -1 : 1;
    }

    public void setDistancePerPulse(double value) {
        this.distancePerPulse = value;
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
