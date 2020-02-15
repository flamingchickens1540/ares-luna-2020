package org.team1540.robot2020;

import edu.wpi.first.wpilibj.I2C;
import edu.wpi.first.wpilibj.SPI;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import org.team1540.robot2020.utils.LIDARLite;
import org.team1540.robot2020.utils.NavX;
import org.team1540.rooster.wrappers.Limelight;

public class LocalizationManager extends CommandBase {
    // TODO: Eventually, this should have its hardware compleately private and just return transforms
    private Limelight limelight = new Limelight("limelight-a");
    private LIDARLite lidar = new LIDARLite(I2C.Port.kOnboard);
    private NavX navx = new NavX(SPI.Port.kMXP);

    public LocalizationManager() {
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

    @Override
    public void execute() {
        SmartDashboard.putNumber("telemetry/lidarDistance", lidar.getDistance());
        SmartDashboard.putNumber("telemetry/angleRadians", navx.getAngleRadians());
        SmartDashboard.putNumber("telemetry/yawRadians", navx.getYawRadians());
    }

    @Override
    public boolean runsWhenDisabled() {
        return true;
    }
}
