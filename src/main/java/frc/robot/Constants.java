package frc.robot;

public final class Constants {
    public static class OperatorConstants {
        public static final int RIGHT_JOYSTICK_PORT = 0;
        public static class ControllerConstants {
        }
    }

    /**
     * Key: RIO = RoboRio, COD = CANCoder, DRI = Drive Motor, ROT = Rotation Motor
     * USED:
     * 0  (RIO)
     * 1  (COD)
     * 2  (DRI)
     * 3  (COD)
     * 4  (DRI)
     * 5  (ROT)
     * 6  (COD)
     * 7  (ROT)
     * 8  (DRI)
     * 9  (DRI)
     * 10 (ROT)
     * 11 (ROT)
     * 12 (COD)
     */
    public static class PortConstants {
        public static final int FRONT_LEFT_DRIVE_MOTOR_ID           = 2;
        public static final int FRONT_RIGHT_DRIVE_MOTOR_ID          = 4;
        public static final int BACK_LEFT_DRIVE_MOTOR_ID            = 9;
        public static final int BACK_RIGHT_DRIVE_MOTOR_ID           = 8;
        public static final int FRONT_LEFT_ROTATION_MOTOR_ID        = 10;
        public static final int FRONT_RIGHT_ROTATION_MOTOR_ID       = 5;
        public static final int BACK_LEFT_ROTATION_MOTOR_ID         = 11;
        public static final int BACK_RIGHT_ROTATION_MOTOR_ID        = 7;
        public static final int FRONT_LEFT_ROTATION_CANCODER_ID     = 12;
        public static final int FRONT_RIGHT_ROTATION_CANCODER_ID    = 3;
        public static final int BACK_LEFT_ROTATION_CANCODER_ID      = 6;
        public static final int BACK_RIGHT_ROTATION_CANCODER_ID     = 1;

    }

    public static class DriveConstants {
        public static final double TRAJ_X_CONTROLLER_KP = 0;
        public static final double TRAJ_Y_CONTROLLER_KP = 0;
        public static final double TRAJ_THETA_CONTROLLER_KP = 0;
        public static final double TRAJ_MAX_ANG_VELO = 0;
        public static final double TRAJ_MAX_ANG_ACCEL = 0;
    }

    public static class RobotConstants {
        public static final double SIDE_LENGTH_INCHES = 30; // square

        public static final double DIAGONAL_LENGTH_INCHES = Math.sqrt(2) * SIDE_LENGTH_INCHES;
        public static final double DIAGONAL_LENGTH_CM = DIAGONAL_LENGTH_INCHES * 2.54;
        public static final double SWERVE_MODULE_INSET_FROM_CORNER_CM = 9; // CM

        public static final double SWERVE_MODULE_DIST_FROM_MIDDLE_CM = DIAGONAL_LENGTH_CM - SWERVE_MODULE_INSET_FROM_CORNER_CM;
        public static final double LEG_LENGTHS_CM = SWERVE_MODULE_DIST_FROM_MIDDLE_CM / Math.sqrt(2);
        public static final double LEG_LENGTHS_M = LEG_LENGTHS_CM / 100;
    }

    private Constants() {
        throw new UnsupportedOperationException("This is a utility class and cannot be instantiated");
    }
}
