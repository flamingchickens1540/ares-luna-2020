package org.team1540.robot2020.utils;

import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import org.apache.commons.math3.geometry.euclidean.twod.Vector2D;
import org.team1540.rooster.triggers.AxisButton;
import org.team1540.rooster.triggers.DPadAxis;
import org.team1540.rooster.triggers.MultiAxisButton;
import org.team1540.rooster.triggers.StrictDPadButton;

import java.util.HashMap;
import java.util.Map;
import java.util.function.DoubleSupplier;
import java.util.function.Supplier;

import static org.team1540.rooster.util.ControlUtils.deadzone;
import static org.team1540.rooster.util.MathUtils.preserveSignRaiseToPower;

public class ChickenXboxController {

    XboxController controller;

    /**
     * Construct an instance of a joystick. The joystick index is the USB port on the drivers
     * station.
     *
     * @param port The port on the Driver Station that the joystick is plugged into.
     */
    public ChickenXboxController(int port) {
        this.controller = new XboxController(port);
    }

    public enum Hand {
        LEFT(0),
        RIGHT(1);


        @SuppressWarnings("MemberName")
        public final int value;
        @SuppressWarnings("PMD.UseConcurrentHashMap")
        private static final Map<Integer, Hand> map = new HashMap<>();

        Hand(int value) {
            this.value = value;
        }

        static {
            for (Hand axisId : Hand.values()) {
                map.put(axisId.value, axisId);
            }
        }

        public static Hand of(int value) {
            return map.get(value);
        }
    }

    public enum XboxAxis {
        LEFT_X(0, 1, true),
        LEFT_Y(1, 0, true),
        LEFT_TRIG(2, 2, false),
        RIGHT_TRIG(3, 3, false),
        RIGHT_X(4, 5, true),
        RIGHT_Y(5, 4, true);

        @SuppressWarnings("MemberName")
        public final int value;
        public final int remappedTo;
        public final boolean inverted;
        @SuppressWarnings("PMD.UseConcurrentHashMap")
        private static final Map<Integer, XboxAxis> map = new HashMap<>();

        XboxAxis(int value, int remappedTo, boolean inverted) {
            this.value = value;
            this.remappedTo = remappedTo;
            this.inverted = inverted;
        }

        static {
            for (XboxAxis axisId : XboxAxis.values()) {
                map.put(axisId.value, axisId);
            }
        }

        public static XboxAxis of(int value) {
            return map.get(value);
        }

        public static XboxAxis handX(Hand hand) {
            return hand == Hand.LEFT ? LEFT_X : RIGHT_X;
        }

        public static XboxAxis handY(Hand hand) {
            return hand == Hand.LEFT ? LEFT_Y : RIGHT_Y;
        }

        public static XboxAxis handTrig(Hand hand) {
            return hand == Hand.LEFT ? LEFT_TRIG : RIGHT_TRIG;
        }
    }

    public enum XboxButton {
        A(1),
        B(2),
        X(3),
        Y(4),
        LEFT_BUMPER(5),
        RIGHT_BUMPER(6),
        BACK(7),
        START(8),
        LEFT_PRESS(9),
        RIGHT_PRESS(10);

        @SuppressWarnings("MemberName")
        public final int value;
        @SuppressWarnings("PMD.UseConcurrentHashMap")
        private static final Map<Integer, XboxButton> map = new HashMap<>();

        XboxButton(int value) {
            this.value = value;
        }

        static {
            for (XboxButton axisId : XboxButton.values()) {
                map.put(axisId.value, axisId);
            }
        }

        public static XboxButton of(int value) {
            return map.get(value);
        }

        public static XboxButton handBumper(Hand hand) {
            return hand == Hand.LEFT ? LEFT_BUMPER : RIGHT_BUMPER;
        }

        public static XboxButton handPress(Hand hand) {
            return hand == Hand.LEFT ? LEFT_PRESS : RIGHT_PRESS;
        }
    }

    public StrictDPadButton getButton(DPadAxis button) {
        return new StrictDPadButton(controller, 0, button);
    }

    public JoystickButton getButton(XboxButton button) {
        return new JoystickButton(controller, button.value);
    }

    public AxisButton getButton(double threshold, XboxAxis axis) {
        return new AxisButton(controller, threshold, axis.remappedTo);
    }

    public MultiAxisButton getButton(double threshold, XboxAxis... axes) {
        int[] axesIds = new int[axes.length];
        for (int i = 0; i < axes.length; i++) {
            axesIds[i] = axes[i].remappedTo;
        }
        return new MultiAxisButton(controller, threshold, axesIds);
    }

    public Axis getAxis(XboxAxis axis) {
        return ((Axis) () -> controller.getRawAxis(axis.remappedTo)).inverted(axis.inverted);
    }

    public Axis2D getAxis2D(Hand hand) {
        return () -> new Vector2D(getAxis(XboxAxis.handX(hand)).value(), getAxis(XboxAxis.handY(hand)).value());
    }

    public interface Axis extends DoubleSupplier {

        Axis ZERO = () -> 0;

        default Axis withDeadzone(double deadzone) {
            return () -> deadzone(value(), deadzone);
        }

        default Axis powerScaled(double power) {
            return () -> preserveSignRaiseToPower(value(), power);
        }

        default Axis inverted() {
            return inverted(true);
        }

        default Axis inverted(boolean invert) {
            return invert ? () -> -value() : this::value;
        }

        default double value() {
            return getAsDouble();
        }
    }

    public interface Axis2D extends Supplier<Vector2D> {

        default Axis magnitude() {
            return () -> value().getNorm();
        }

        default Axis angle() {
            return () -> {
                Vector2D value = value(); // to avoid creating the object twice
                return Math.atan2(value.getY(), value.getX());
            };
        }

        default Axis x() {
            return () -> value().getX();
        }

        default Axis y() {
            return () -> value().getY();
        }

        default Axis2D withDeadzone(double deadzone) {
            // TODO this is a square deadzone, make a circular deadzone
            return () -> new Vector2D(deadzone(value().getX(), deadzone), deadzone(value().getY(), deadzone));
        }

        default Vector2D value() {
            return get();
        }
    }
}
