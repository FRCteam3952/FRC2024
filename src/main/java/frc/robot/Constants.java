package frc.robot;

import frc.robot.util.ControlHandler.TriggerType;

public final class Constants {
    public static class NetworkTablesConstants {
        public static final String MAIN_TABLE_NAME = "robot";
    }
    public static class OperatorConstants {
        public static final int RIGHT_JOYSTICK_PORT = 0;
        public static final int NINTENDO_PRO_CONTROLLER = 1;
        public static final int PS5_CONTROLLER = 3;
        public static class ControllerConstants {
            public static final TriggerType RUN_INTAKE = TriggerType.RIGHT_BUTTON;
            public static final TriggerType REVERSE_INTAKE = TriggerType.LOWER_BUTTON;
        }
    }

    /**
     * Key: RIO = RoboRio, COD = CANCoder, DRI = Drive Motor, ROT = Rotation Motor, INT = Intake-related, 
     * USED:
     * 0  (RIO)
     * 1  (COD)
     * 2  (ROT)
     * 3  (COD)
     * 4  (DRI)
     * 5  (ROT)
     * 6  (COD)
     * 7  (DRI)
     * 8  (ROT)
     * 9  (DRI)
     * 10 (ROT)
     * 11 (DRI)
     * 12 (COD)
     * 13 (INT)
     * 14 (INT)
     * 15 (INT)
     */
    public static class PortConstants {
        // CAN
        public static final int TOP_INTAKE_MOTOR_ID                 = 14; //THE MEANING OF LIFE -- change later
        public static final int BOTTOM_INTAKE_MOTOR_ID              = 15; //bad scawy number -- change later
        public static final int INTAKE_PIVOT_MOTOR_ID               = 13; //the age to vote - change
        

        public static final int FRONT_LEFT_DRIVE_MOTOR_ID           = 9;
        public static final int FRONT_RIGHT_DRIVE_MOTOR_ID          = 11;
        public static final int BACK_LEFT_DRIVE_MOTOR_ID            = 7;
        public static final int BACK_RIGHT_DRIVE_MOTOR_ID           = 4;

        public static final int FRONT_LEFT_ROTATION_MOTOR_ID        = 8;
        public static final int FRONT_RIGHT_ROTATION_MOTOR_ID       = 2;
        public static final int BACK_LEFT_ROTATION_MOTOR_ID         = 5;
        public static final int BACK_RIGHT_ROTATION_MOTOR_ID        = 10;

        public static final int FRONT_LEFT_ROTATION_CANCODER_ID     = 12;
        public static final int FRONT_RIGHT_ROTATION_CANCODER_ID    = 3;
        public static final int BACK_LEFT_ROTATION_CANCODER_ID      = 6;
        public static final int BACK_RIGHT_ROTATION_CANCODER_ID     = 1;


        // DIO
        public static final int INTAKE_DOWN_LIMIT_SWITCH_PORT       = 1;
        public static final int INTAKE_UP_LIMIT_SWITCH_PORT         = 3;
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
