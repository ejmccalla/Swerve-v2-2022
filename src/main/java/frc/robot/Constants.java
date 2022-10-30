package frc.robot;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.util.Units;

/**
 * The Constants class constains all of the port mappings and other numerical values which don't
 * require calibration.
 */
public final class Constants {


    /**
     * The DriverStation constants are port numbers used by the joysticks/button/controllers on the
     * drivers station.
     */
    public static final class Driver {
        public static final double MAX_DRIVE_VELOCITY_MPS = 3.0;
        public static final double MAX_ROTATION_VELOCITY_RPS = 10.0;
    }

    /**
     * The DriverStation constants are port numbers used by the joysticks/button/controllers on the
     * drivers station.
     */
    public static final class DriverStation {
        public static final int LEFT_JOYSTICK = 0;
        public static final int RIGHT_JOYSTICK = 1;
        public static final int DRIVER_BUTTON_BOARD = 2;
    }

    /**
     * The Hardware constants are port numbers and/or CAN ID's of the robot hardware which don't
     * belong to an explicit sub-system.
     */
    public static final class Hardware {
        public static final int PDP_ID = 40;
        public static final int PCM_ID = 12;
    }

    /**
     * The Intake constants are port numbers and/or CAN ID's of the robot hardware which don't
     * belong to an explicit sub-system.
     */
    public static final class Intake {
        /** Switch to enable logging telemetry to disk. */
        public static final boolean ENABLE_LOGGING = true;
        /** CAN ID of the intake motor. */
        public static final int MOTOR_ID = 0;
        /** Port ID of the solenoid. */
        public static final int SOLENOID_ID = 0;
        /** Gear reduction. */
        public static final double GEAR_RATIO = 1.0 / 3.0;
        /** Roller diameter. */
        public static final double ROLLER_DIAMETER_FT = 1.1 / 12.0;
    }

    /**
     * The Drivetrain constants are port numbers and/or CAN ID's of the robot hardware which don't
     * belong to an explicit sub-system.
     *
     * <p>The positions of the modules are relative to the robot center.
     */
    public static final class Drivetrain {
        /** Switch to enable logging telemetry to disk. */
        public static final boolean ENABLE_LOGGING = true;
        /** Module order: front left, front right, rear left, rear right. */
        public static final String[] MODULE_LABELS = {"FL", "FR", "RL", "RR"};
        /** Module order: front left, front right, rear left, rear right. */
        public static final int[] TURN_IDS = {9, 30, 11, 18};
        /** Module order: front left, front right, rear left, rear right. */
        public static final int[] DRIVE_IDS = {8, 1, 10, 19};
        /** Module order: front left, front right, rear left, rear right. */
        public static final int[] QUAD_A_DIO_CHANNELS = {7, 10, 4, 1};
        /** Module order: front left, front right, rear left, rear right. */
        public static final int[] QUAD_B_DIO_CHANNELS = {8, 11, 5, 2};
        /** Module order: front left, front right, rear left, rear right. */
        public static final int[] PWM_DIO_CHANNELS = {6, 9, 3, 0};

        /** https://www.swervedrivespecialties.com/products/mk4-swerve-module#:~:text=The%20steering%20gear%20ratio%20of,4%20different%20drive%20gear%20ratios */
        public static final double TURN_GEAR_RATIO = 12.8;
        public static final double DRIVE_GEAR_RATIO = 6.75;

        /** Module order: front left, front right, rear left, rear right. */
        public static final Translation2d[] MODULE_LOCATIONS = {
            new Translation2d(Units.inchesToMeters(Calibrations.WHEEL_BASE_INCH) / 2,
                              Units.inchesToMeters(Calibrations.TRACK_WIDTH_INCH) / 2),
            new Translation2d(Units.inchesToMeters(Calibrations.WHEEL_BASE_INCH) / 2,
                              Units.inchesToMeters(Calibrations.TRACK_WIDTH_INCH) / 2),
            new Translation2d(-Units.inchesToMeters(Calibrations.WHEEL_BASE_INCH) / 2,
                              Units.inchesToMeters(Calibrations.TRACK_WIDTH_INCH) / 2),
            new Translation2d(-Units.inchesToMeters(Calibrations.WHEEL_BASE_INCH) / 2,
                              -Units.inchesToMeters(Calibrations.TRACK_WIDTH_INCH) / 2)
        };

        public static final double turnEncPpr = 4096.0;
        public static final int numModules = 4;

    }

}
