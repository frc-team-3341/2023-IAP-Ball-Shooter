package frc.robot;

public class Constants {
    public static final int joystick = 0;
    public static final int feedForward = 0;
    public static final int speed = 0;
    public static final int buttons = 0;
    public static final int circumference = 0;

    public static final class BallHandlerPorts {
        public static final int beamBreakPort = 1;
        public static final int flywheelPort = 4;
        public static final int FeedwheelPort = 3;
    }

    public static final class leftFlywheelPIDConsts {
        public static double pidP = 0.065619;
        public static double pidI = 0;
        public static double pidD = 0;
    }

    public static final class pivotPIDConsts {
        public static final double pidP = 0.05;
        public static final double pidI = 0;
        public static final double pidD = 0;
}
    public static final class feedForwardConsts {
        public static final double kS = 0.80824;
        public static final double kV = 0.12407;
        public static final double kA = 0.015099;
    }
        public static final double flyCircumference = 31.42;
    }
