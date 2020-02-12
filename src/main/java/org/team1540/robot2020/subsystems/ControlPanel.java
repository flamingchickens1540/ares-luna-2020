package org.team1540.robot2020.subsystems;

import com.revrobotics.ColorSensorV3;
import edu.wpi.first.wpilibj.I2C;
import edu.wpi.first.wpilibj.Servo;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class ControlPanel extends SubsystemBase {

    private Servo armServo = new Servo(8);
    private final ColorSensorV3 m_colorSensor = new ColorSensorV3(I2C.Port.kOnboard);


    public ControlPanel() {
        setArmServo(true);
    }

    public enum ControlPanelServoState {
        UP(0.9137059647675217),
        PANEL(0.402392418539555),
        DOWN(0);

        private double servoPosition;

        ControlPanelServoState(double servoPosition) {
            this.servoPosition = servoPosition;
        }
    }

    public void setArmServo(boolean state) {
        armServo.set(state ? ControlPanelServoState.PANEL.servoPosition : ControlPanelServoState.UP.servoPosition);
    }

    public void setArmServo(double position) {
        armServo.set(position);
    }
}
