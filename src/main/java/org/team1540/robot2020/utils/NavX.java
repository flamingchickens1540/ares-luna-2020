package org.team1540.robot2020.utils;

import com.kauailabs.navx.frc.AHRS;
import com.kauailabs.navx.frc.ITimestampedDataSubscriber;
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
        return radiansFromNavXRaw(navx.getYaw());
    }

    /**
     * @return NavX angle counter-clockwise in radians. This method continues past pi and -pi and is thus the one you don't want to use (most of the time).
     */
    public double getAngleRadians() {
        return radiansFromNavXRaw(navx.getAngle());
    }

    public double getRate() {
        return radiansFromNavXRaw(navx.getRate());
    }

    public void zeroYaw() {
        navx.zeroYaw();
    }

    public void registerCallback(ITimestampedDataSubscriber callback, Object callback_context) {
        navx.registerCallback(callback, callback_context);
    }

    public void deregisterCallback(ITimestampedDataSubscriber callback) {
        navx.deregisterCallback(callback);
    }

    public static double radiansFromNavXRaw(double navxYaw) {
        return -Math.toRadians(navxYaw);
    }

    public double getPitch() {
        return radiansFromNavXRaw(navx.getPitch());
    }
}
