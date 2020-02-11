package org.team1540.robot2020.commands;

import com.revrobotics.ColorSensorV3;
import edu.wpi.first.wpilibj.I2C;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj2.command.CommandBase;
import org.team1540.robot2020.utils.LIDARLite;

public class I2CTest extends CommandBase {
    private ColorSensorV3 colorSensor = new ColorSensorV3(I2C.Port.kOnboard);
    private LIDARLite lidar = new LIDARLite(I2C.Port.kOnboard);


    public I2CTest() {
    }

    /**
     * Return RGB string of color
     *
     * @param color Color
     * @return R, G, B
     */
    private static String _toString(Color color) {
        return color.red + "," + color.green + "," + color.blue;
    }

    @Override
    public void initialize() {
        lidar.startMeasuring();
    }

    @Override
    public void execute() {
        SmartDashboard.putNumber("telemetry/lidarDistance", lidar.getDistance());
        SmartDashboard.putString("telemetry/color", _toString(colorSensor.getColor()));
    }

    @Override
    public boolean runsWhenDisabled() {
        return true;
    }
}
