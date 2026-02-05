package frc.robot;

public final class Constants {

        public static final class SwerveConstants {

        public static final int defaultValue = 0;
        public static final int PIGEON_2 = 22;
        public static final int FL_DRIVE = 1;
        public static final int FL_STEER = 2;
        public static final int FR_DRIVE = 3;
        public static final int FR_STEER = 4;
        public static final int BL_DRIVE = 5;
        public static final int BL_STEER = 6;
        public static final int BR_DRIVE = 7;
        public static final int BR_STEER = 8;
        public static final double FL_OFFSET = -0.1435546875;
        public static final double FR_OFFSET = 0.3017578125;
        public static final double BL_OFFSET = 0.208251953125;
        public static final double BR_OFFSET = -0.353759765625;
        public static final double WHEEL_BASE = 0.55; 
        public static final double TRACK_WIDTH = 0.55;
        public static final double MAX_SPEED = 8.0;
        public static final double MAX_ANGULAR_SPEED = 20 * Math.PI;
        public static final double STEER_kP = 4.0;
        public static final double STEER_kI = 0.0;
        public static final double STEER_kD = 0.1;
        public static final double SLEW_RATE_LIMITER_X = 18.0;
        public static final double SLEW_RATE_LIMITER_Y = 18.0;
        public static final double SLEW_RATE_LIMITER_ROT = 40.0;
        
    }

    public static final class JoystickConstants {
    
        public static final int joystickPort1 = 0; 
        public static final double DEADBAND = 0.05;
        public static final int A = 2;
        public static final int B = 3;
        public static final int X = 1;
        public static final int Y = 4;
        public static final int lB = 5;
        public static final int rB = 6;
        public static final int lT = 7;
        public static final int rT = 8;
        public static final int BACK = 9;
        public static final int START = 10;
        public static final int L_STICK = 11;
        public static final int R_STICK = 12;
   
    }

}