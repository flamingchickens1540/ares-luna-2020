package org.team1540.robot2020.commands.drivetrain;

public class PIDConfig {

    public double p;
    public double i;
    public double d;
    public double iMax;
    public double outputMax;
    public double c;

    public PIDConfig(double p, double i, double d, double iMax, double outputMax, double c) {
        this.p = p;
        this.i = i;
        this.d = d;
        this.iMax = iMax;
        this.outputMax = outputMax;
        this.c = c;
    }
}
