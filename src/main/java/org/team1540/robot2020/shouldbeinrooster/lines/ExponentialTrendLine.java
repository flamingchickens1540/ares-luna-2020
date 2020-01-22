package org.team1540.robot2020.shouldbeinrooster.lines;

public class ExponentialTrendLine extends OLSTrendLine {
    @Override
    protected double[] xVector(double x) {
        return new double[]{1, x};
    }

    @Override
    protected boolean logY() {
        return true;
    }
}
