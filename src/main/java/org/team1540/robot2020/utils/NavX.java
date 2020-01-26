package org.team1540.robot2020.utils;

import com.kauailabs.navx.frc.AHRS;
import edu.wpi.first.wpilibj.SPI.Port;

public class NavX {

    private final AHRS navx;

    public NavX(Port port) {
        navx = new AHRS(port);
    }

    /**
     * @return NavX yaw counter-clockwise in radians, from -pi to pi. This method does NOT continue past pi or -pi and is thus the one you probably want to use most of the time.
     */
    public double getYawRadians() {
        return -Math.toRadians(navx.getYaw());
    }

    /**
     * @return NavX angle counter-clockwise in radians. This method continues past pi and -pi and is thus the one you don't want to use (most of the time).
     */
    public double getAngleRadians() {
        return -Math.toRadians(navx.getAngle());
    }
}
