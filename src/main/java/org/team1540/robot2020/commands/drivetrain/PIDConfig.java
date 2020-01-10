package org.team1540.robot2020.commands.drivetrain;

public class PIDConfig {

    public double p;
    public double i;
    public double d;
    public double IMax;

    public PIDConfig(double p, double i, double d, double IMax) {
        this.p = p;
        this.i = i;
        this.d = d;
        this.IMax = IMax;
    }
}
