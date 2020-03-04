package org.team1540.robot2020.shouldbeinrooster;

import edu.wpi.first.networktables.EntryListenerFlags;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import org.team1540.rooster.util.ChickenXboxController;

import java.util.HashMap;
import java.util.Map;

public class MotorTesting {
    private static final MotorTesting INSTANCE = new MotorTesting();

    private Map<Integer, GenericMotor> motors = new HashMap<>();
    public int testIndex = 0;

    public void addMotor(GenericMotor motor) {
        motors.put(motor.index, motor);
    }

    public void testMotors(ChickenXboxController.Axis axis) {
        indexToShuffleboard();
//        NetworkTableInstance.getDefault().getTable("SmartDashboard/motorTesting/motor")
//                .addEntryListener((table, key, entry, value, flags) ->
//                        testIndex = (int) SmartDashboard.getNumber("motorTesting
//                        /motor", 0), EntryListenerFlags.kUpdate);
        new TestMotor(() -> motors.get(testIndex), axis).schedule();

//        sort
//        Set<String> keys = motors.keySet();
//        SmartDashboard.putStringArray("motorTesting/motors", keys.toArray(new String[keys.size()]));
//        new TestMotor(() -> motors.get(SmartDashboard.getString("motorTesting/motors", "driveLeftA")), axis);
    }

    public boolean hasMotor(int index) {
        return motors.containsKey(index);
    }

    public void indexToShuffleboard() {
        SmartDashboard.putNumber("motorTesting/motor", testIndex);
    }

    public static MotorTesting getInstance() {
        return INSTANCE;
    }
}
