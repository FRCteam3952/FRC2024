package frc.robot;

import frc.robot.util.ControlHandler.TriggerType;

/**
 * Intended for constants on the robot that should rarely change (especially in a competition scenario). These values tend to be primitive values (but are not necessarily).
 * Examples of these values are ports or IDs where motors are plugged in.
 * This is different from the {@link frc.robot.Flags Flags} class, which toggles functionality on the robot and may be changed more often.
 */
public final class Constants {
    private Constants() {
        throw new UnsupportedOperationException("This is a utility class and cannot be instantiated");
    }

    public static class NetworkTablesConstants {
        public static final String MAIN_TABLE_NAME = "robot";
    }

    public static class OperatorConstants {
        public static final int RIGHT_JOYSTICK_PORT = 0;
        public static final int NINTENDO_PRO_CONTROLLER = 1;
        public static final int PS5_CONTROLLER = 3;

        // Should this be here? especially with our new controller system, we could potentially refactor or re-abstract this using another class (maybe even for multiple driver preferences?)
        public static class ControllerConstants {
            public static final TriggerType ZERO_GYRO = TriggerType.LEFT_BUTTON;
            public static final TriggerType ZERO_SWERVE_MODULES = TriggerType.UPPER_BUTTON;

            public static final TriggerType INTAKE_RUN = TriggerType.RIGHT_BUTTON;
            public static final TriggerType INTAKE_REVERSE = TriggerType.LOWER_BUTTON;
            public static final TriggerType INTAKE_POS_TOGGLE = TriggerType.LEFT_SHOULDER_BUTTON;

            public static final TriggerType SHOOTER_RUN = TriggerType.RIGHT_SHOULDER_TRIGGER;
        }
    }

    /**
     * Key: RIO = RoboRio, COD = CANCoder, DRI = Drive Motor, ROT = Rotation Motor, INT = Intake, CON = Conveyor, CLI = Climber, SHO = Shooter
     * CAN IDs USED:
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
     * 15 (CON)
     * 16 (SHO)
     * 17 (SHO)
     * 18 (CON)
     * 19 (CLI)
     * 20 (CLI)
     * 21 (SHO)
     * 22 (CON)
     * 23 (INT)
     * <p>
     * 
     * DIOs USED:
     * 0 (SHO)
     * 1 (SHO)
     * 2 (SHO)
     * 3 (INT)
     * 4 (INT)
     * 5 (INT)
     * 
     * PWMs USED:
     * 0 (SHO)
     * 1 (SHO)
     */
    public static class PortConstants {
        // CAN IDs

        // Drive Train
        public static final int DTRAIN_FRONT_LEFT_DRIVE_MOTOR_ID        = 9;
        public static final int DTRAIN_FRONT_RIGHT_DRIVE_MOTOR_ID       = 11;
        public static final int DTRAIN_BACK_LEFT_DRIVE_MOTOR_ID         = 7;
        public static final int DTRAIN_BACK_RIGHT_DRIVE_MOTOR_ID        = 4;

        public static final int DTRAIN_FRONT_LEFT_ROTATION_MOTOR_ID     = 8;
        public static final int DTRAIN_FRONT_RIGHT_ROTATION_MOTOR_ID    = 2;
        public static final int DTRAIN_BACK_LEFT_ROTATION_MOTOR_ID      = 5;
        public static final int DTRAIN_BACK_RIGHT_ROTATION_MOTOR_ID     = 10;

        public static final int DTRAIN_FRONT_LEFT_CANCODER_ID           = 12;
        public static final int DTRAIN_FRONT_RIGHT_CANCODER_ID          = 3;
        public static final int DTRAIN_BACK_LEFT_CANCODER_ID            = 6;
        public static final int DTRAIN_BACK_RIGHT_CANCODER_ID           = 1;

        // Intake
        public static final int INTAKE_TOP_MOTOR_ID                     = 13;
        public static final int INTAKE_BOTTOM_MOTOR_ID                  = 14;
        public static final int INTAKE_PIVOT_MOTOR_ID                   = 23;

        // Conveyor
        public static final int CONVEYOR_TO_SHOOTER_MOTOR_ID            = 22;
        public static final int CONVEYOR_LEFT_MOTOR_ID                  = 18;
        public static final int CONVEYOR_RIGHT_MOTOR_ID                 = 15;

        // Shooter
        public static final int SHOOTER_LEFT_MOTOR_ID                   = 21;
        public static final int SHOOTER_RIGHT_MOTOR_ID                  = 16;
        public static final int SHOOTER_PIVOT_MOTOR_ID                  = 17;

        // Climber
        public static final int CLIMBER_LEFT_MOTOR_ID                   = 20;
        public static final int CLIMBER_RIGHT_MOTOR_ID                  = 19;

        // DIO Ports

        // Intake
        public static final int INTAKE_ABSOLUTE_ENCODER_ABS_PORT        = 3;
        public static final int INTAKE_ABSOLUTE_ENCODER_A_PORT          = 4;
        public static final int INTAKE_ABSOLUTE_ENCODER_B_PORT          = 5;
        
        // Shooter
        public static final int SHOOTER_ABSOLUTE_ENCODER_ABS_PORT       = 0;
        public static final int SHOOTER_ABSOLUTE_ENCODER_A_PORT         = 1;
        public static final int SHOOTER_ABSOLUTE_ENCODER_B_PORT         = 2;

        
        // PWM Ports

        // Shooter
        public static final int SHOOTER_LEFT_SERVO_PORT                 = 1;
        public static final int SHOOTER_RIGHT_SERVO_PORT                = 0;
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
}
