package org.team1540.robot2020;

import edu.wpi.first.wpilibj.I2C;
import edu.wpi.first.wpilibj.SPI;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import org.team1540.robot2020.utils.LIDARLite;
import org.team1540.robot2020.utils.NavX;
import org.team1540.rooster.wrappers.Limelight;

public class TransformManager {
    private Limelight limelight = new Limelight("limelight");
    private LIDARLite lidar = new LIDARLite(I2C.Port.kOnboard);
    private NavX navx = new NavX(SPI.Port.kMXP);

    public TransformManager() {
    }

    public NavX getNavX() {
        return navx;
    }
    public Limelight getLimelight() {
        return limelight;
    }
    public LIDARLite getLidar(){
        return lidar;
    }

    public void initialize() {
        lidar.startMeasuring();
    }

    public void periodic() {
        SmartDashboard.putNumber("telemetry/lidarDistance", lidar.getDistance());
        SmartDashboard.putNumber("telemetry/angleRadians", navx.getAngleRadians());
        SmartDashboard.putNumber("telemetry/yawRadians", navx.getYawRadians());
    }
}
