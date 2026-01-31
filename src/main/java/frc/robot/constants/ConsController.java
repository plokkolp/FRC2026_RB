package frc.robot.constants;

public class ConsController {
    public static final int kDriveControllerPort = 0;
    public static final int kOperatorControllerPort = 1;
    public static final int kButtonBroadControllerPort = 2;
    public static final double DEADBAND = 0.05;
    public enum Axis {
        LEFT_STICK_X(0),
        LEFT_STICK_Y(1),
        LEFT_TRIGGER(2),
        RIGHT_TRIGGER(3),
        RIGHT_STICK_X(4),
        RIGHT_STICK_Y(5);
        public final int id;
        Axis(int id) {this.id = id;}
    }
    
    public enum Button {
        BUTTON_A(1),
        BUTTON_B(2),
        BUTTON_X(3),
        BUTTON_Y(4),
        BUTTON_START(8),
        BUTTON_BACK(7),
        BUTTON_RB(6),
        BUTTON_LB(5),
        BUTTON_C(9),
        BUTTON_Z(10);
        public final int id;
        Button(int id) {this.id = id;}
    }
}
