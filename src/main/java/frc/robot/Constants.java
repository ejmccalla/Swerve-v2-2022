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
    public static final class DriverStation {
        public static final int LEFT_JOYSTICK = 0;
        public static final int RIGHT_JOYSTICK = 1;
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
     * The Drivetrain constants are port numbers and/or CAN ID's of the robot hardware which don't
     * belong to an explicit sub-system.
     *
     * <p>The positions of the modules are relative to the robot center.
     */
    public static final class Drivetrain {
        public static final int FRONT_LEFT_TURN_ID = 9;
        public static final int FRONT_LEFT_DRIVE_ID = 10;
        public static final int FRONT_LEFT_QUAD_A_DIO_CHANNEL = 7;
        public static final int FRONT_LEFT_QUAD_B_DIO_CHANNEL = 8;
        public static final int FRONT_LEFT_PWM_DIO_CHANNEL = 6;
        public static final int FRONT_RIGHT_TURN_ID = 30;
        public static final int FRONT_RIGHT_DRIVE_ID = 1;
        public static final int FRONT_RIGHT_QUAD_A_DIO_CHANNEL = 10;
        public static final int FRONT_RIGHT_QUAD_B_DIO_CHANNEL = 11;
        public static final int FRONT_RIGHT_PWM_DIO_CHANNEL = 9;
        public static final int REAR_LEFT_TURN_ID = 11;
        public static final int REAR_LEFT_DRIVE_ID = 10;
        public static final int REAR_LEFT_QUAD_A_DIO_CHANNEL = 4;
        public static final int REAR_LEFT_QUAD_B_DIO_CHANNEL = 5;
        public static final int REAR_LEFT_PWM_DIO_CHANNEL = 3;
        public static final int REAR_RIGHT_TURN_ID = 18;
        public static final int REAR_RIGHT_DRIVE_ID = 19;
        public static final int REAR_RIGHT_QUAD_A_DIO_CHANNEL = 1;
        public static final int REAR_RIGHT_QUAD_B_DIO_CHANNEL = 2;
        public static final int REAR_RIGHT_PWM_DIO_CHANNEL = 0;

        /** https://www.swervedrivespecialties.com/products/mk4-swerve-module#:~:text=The%20steering%20gear%20ratio%20of,4%20different%20drive%20gear%20ratios */
        public static final double TURN_GEAR_RATIO = 12.8;
        public static final double DRIVE_GEAR_RATIO = 6.75;

        public static final Translation2d FRONT_LEFT_LOCATION =
            new Translation2d(Units.inchesToMeters(Calibrations.WHEEL_BASE_INCH) / 2,
                              Units.inchesToMeters(Calibrations.TRACK_WIDTH_INCH) / 2);
        public static final Translation2d FRONT_RIGHT_LOCATION =
            new Translation2d(Units.inchesToMeters(Calibrations.WHEEL_BASE_INCH) / 2,
                              Units.inchesToMeters(Calibrations.TRACK_WIDTH_INCH) / 2);
        public static final Translation2d REAR_LEFT_LOCATION =
            new Translation2d(-Units.inchesToMeters(Calibrations.WHEEL_BASE_INCH) / 2,
                              Units.inchesToMeters(Calibrations.TRACK_WIDTH_INCH) / 2);
        public static final Translation2d REAR_RIGHT_LOCATION =
            new Translation2d(-Units.inchesToMeters(Calibrations.WHEEL_BASE_INCH) / 2,
                              -Units.inchesToMeters(Calibrations.TRACK_WIDTH_INCH) / 2);



    }

}
