package org.team1540.robot2020.shouldbeinrooster;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import org.team1540.rooster.util.ChickenXboxController;

import java.util.HashMap;
import java.util.Map;
import java.util.Set;

public class MotorTesting {
    private static final MotorTesting INSTANCE = new MotorTesting();

    private Map<String, GenericMotor> motors = new HashMap<>();

    public void addMotor(GenericMotor motor) {
        motors.put(motor.name, motor);
    }

    public void testMotors(ChickenXboxController.Axis axis) {
//        sort
        Set<String> keys = motors.keySet();
        SmartDashboard.putStringArray("motorTesting/motors", keys.toArray(new String[keys.size()]));
        new TestMotor(() -> motors.get(SmartDashboard.getString("motorTesting/motors", "driveLeftA")), axis);
    }

    public static MotorTesting getInstance() {
        return INSTANCE;
    }
}
