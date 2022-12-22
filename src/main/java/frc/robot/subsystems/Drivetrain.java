package frc.robot.subsystems;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveDriveOdometry;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.util.datalog.BooleanLogEntry;
import edu.wpi.first.util.datalog.DataLog;
import edu.wpi.first.util.datalog.StringLogEntry;
import edu.wpi.first.wpilibj.DataLogManager;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Calibrations;
import frc.robot.Constants;
import frc.robot.lib.ADIS16470;
import frc.robot.lib.SwerveModule;

/**
 * Implements the swerve drivetrain using REV/SparkMax, swerve MK4 modules, SRX mag encoders, and an ADIS16470 IMU.
 *
 * @see <a href="https://www.analog.com/media/en/technical-documentation/data-sheets/ADIS16470.pdf">ADIS16470 IMU Datasheet</a>
 */
public class Drivetrain extends SubsystemBase {

    /**
     * The states of the drivetrain.
     *
     * <p>Commands are responsible for setting the state of the drivetrain and are defined as follows:
     *
     * <p><b>Idle</b> - There are no commands currently using the subsystem.
     *
     * <p><b>Driving</b> - The driver is controlling the robot.
     *
     * <p><b>PathFollowing</b> - The robot is following a path.
     *
     * <p><b>Calibrating</b> - The robot is running a calibration command.
     */
    public enum StateType {
        Idle {
            @Override
            public String toString() {
                return "Idle";
            }
        },
        Driving {
            @Override
            public String toString() {
                return "Driving";
            }
        },
        PathFollowing {
            @Override
            public String toString() {
                return "Path Following";
            }
        },
        Calibrating {
            @Override
            public String toString() {
                return "Calibrating";
            }
        },
    }

    private final SwerveModule[] m_modules;
    private final SwerveDriveKinematics m_kinematics;
    private final ADIS16470 m_imu;
    private final SwerveDriveOdometry m_odometry;
    private final DataLog m_log;
    private SwerveModuleState[] m_currentModulesState;
    private SwerveModuleState[] m_desiredModulesState;
    private ChassisSpeeds m_chassisSpeeds;
    private boolean m_isFieldOriented;
    private Rotation2d m_imuYawAngleRot2D;
    private StateType m_currentState;
    private StringLogEntry m_stateLogEntry;
    private BooleanLogEntry m_fieldOrientedLogEntry;


    //--------------------------------------------------------------------------------------------------------------------//
    /*                                                   PUBLIC METHODS                                                   */
    //--------------------------------------------------------------------------------------------------------------------//


    /**
     * Set the state of the swerve modules to perform the desired auto/teleop command.
     *
     * @param speedX the translational speed along the X-axis
     * @param speedY the translational speed along the Y-axis
     * @param speedRot the rotational speed about the Z-axis
     */
    public void setDesiredModulesState(double speedX, double speedY, double speedRot) {
        if (m_isFieldOriented) {
            m_chassisSpeeds = ChassisSpeeds.fromFieldRelativeSpeeds(speedX, speedY, speedRot, m_imuYawAngleRot2D);
        } else {
            m_chassisSpeeds = new ChassisSpeeds(speedX, speedY, speedRot);
        }
        m_desiredModulesState = m_kinematics.toSwerveModuleStates(m_chassisSpeeds);

        SwerveDriveKinematics.desaturateWheelSpeeds(m_desiredModulesState, Calibrations.MAX_DRIVE_VELOCITY_MPS);
        for (int i = 0; i < Constants.Drivetrain.numModules; i++) {
            m_modules[i].setDesiredState(m_desiredModulesState[i]);
        }
    }


    /**
     * Set offset of the modules azimuth angle by using the input absolute "home" angle and the current angle read from the
     * duty cycle encoder.
     */
    public void setHomeOffsets() {
        for (int i = 0; i < Constants.Drivetrain.numModules; i++) {
            m_modules[i].setHomeOffset(Calibrations.ZEROS_RAD[i]);
        }
    }


    /**
     * Set all of the swerve module motor outputs to 0 volts.
     */
    public void setIdleModules() {
        for (int i = 0; i < Constants.Drivetrain.numModules; i++) {
            m_modules[i].setIdle();
        }
    }


    /**
     * Sets the commanded voltage of the turn motors.
     *
     * @param voltage the voltage input of the motors
     */
    public void setModulesTurnVoltage(double voltage) {
        for (int i = 0; i < Constants.Drivetrain.numModules; i++) {
            m_modules[i].setTurnVoltage(voltage);
        }
    }


    /**
     * Sets the commanded voltage of the drive motors.
     *
     * @param voltage the voltage input of the motors
     */
    public void setModulesDriveVoltage(double voltage) {
        for (int i = 0; i < Constants.Drivetrain.numModules; i++) {
            m_modules[i].setDriveVoltage(voltage);
        }
    }


    /**
     * Set the idle mode of the modules motors.
     *
     * @param isBrakeDesired true sets brake mode, false sets coast mode
     */
    public void setModulesIdleMode(boolean isBrakeDesired) {
        for (int i = 0; i < Constants.Drivetrain.numModules; i++) {
            m_modules[i].setIdleMode(isBrakeDesired);
        }
    }

    /**
     * Set the drive encoder position to 0 for all modules.
     */
    public void resetModulesDriveEncoder() {
        for (int i = 0; i < Constants.Drivetrain.numModules; i++) {
            m_modules[i].resetDriveEncoder();
        }
    }


    /**
     * Get the offset of the modules azimuth angle.
     *
     * @return the home offsets in radians
     */
    public double[] getHomeOffsets() {
        double[] homeOffsets = new double[Constants.Drivetrain.numModules];
        for (int i = 0; i < Constants.Drivetrain.numModules; i++) {
            homeOffsets[i] = m_modules[i].getHomeOffset();
        }
        return homeOffsets;
    }


    /**
     * Get the drive encoder position for all modules in their native units of rotations.
     *
     * @return the rotations of the swerve drive encoders
     */
    public double[] getModulesDriveRotations() {
        double[] rotations = new double[Constants.Drivetrain.numModules];
        for (int i = 0; i < Constants.Drivetrain.numModules; i++) {
            rotations[i] = m_modules[i].getDriveEncPosRot();
        }
        return rotations;
    }


    /**
     * Update the state of the drivetrain.
     *
     * <p>The state will be updated by the commands which use the drivetrain subsystem.
     *
     * @param state the drivetrain state
     */
    public void updateState(StateType state) {
        m_currentState = state;
    }


    /**
     * Toggle whether the drive commands are field oriented or robot oriented.
     */
    public void toggleFieldOriented() {
        m_isFieldOriented = !m_isFieldOriented;
    }

    /**
     * Get the drive command orientation.
     *
     * @return if driving is field oriented
     */
    public boolean isFieldOriented() {
        return m_isFieldOriented;
    }


    /**
     * Get the yaw angle of the IMU.
     *
     * @return the IMU yaw angle in degrees
     */
    public double getImuYawAngleDeg() {
        return m_imu.getAngle();
    }

    /**
     * Get the die temp of the IMU.
     *
     * @return the IMU die temp in degrees celsius
     */
    public double getImuTempDegC() {
        return m_imu.getTemp();
    }


    //--------------------------------------------------------------------------------------------------------------------//
    /*                                                  PRIVATE METHODS                                                   */
    //--------------------------------------------------------------------------------------------------------------------//


    /**
     * Log the telemetry data to disk using the WPILib logger.
     */
    private void logTelemetry() {
        if (Constants.Drivetrain.ENABLE_LOGGING) {
            m_stateLogEntry.append(m_currentState.toString());
            m_fieldOrientedLogEntry.append(isFieldOriented());
        }
    }

    /**
     * Log the telemetry data for all modules.
     */
    private void logModulesTelemetry() {
        for (int i = 0; i < Constants.Drivetrain.numModules; i++) {
            m_modules[i].logTelemetry();
        }
    }

    //--------------------------------------------------------------------------------------------------------------------//
    /*                                         CONSTRUCTOR AND PERIODIC METHODS                                           */
    //--------------------------------------------------------------------------------------------------------------------//


    /** 
     * Constructor for the drivetrain.
     */
    public Drivetrain() {
        m_modules = new SwerveModule[Constants.Drivetrain.numModules];
        m_desiredModulesState = new SwerveModuleState[Constants.Drivetrain.numModules];
        m_currentModulesState = new SwerveModuleState[Constants.Drivetrain.numModules];
        for (int i = 0; i < Constants.Drivetrain.numModules; i++) {
            m_modules[i] = new SwerveModule(Constants.Drivetrain.MODULE_LABELS[i],
                                            Constants.Drivetrain.TURN_IDS[i],
                                            Constants.Drivetrain.DRIVE_IDS[i],
                                            Constants.Drivetrain.PWM_DIO_CHANNELS[i],
                                            Constants.Drivetrain.QUAD_A_DIO_CHANNELS[i],
                                            Constants.Drivetrain.QUAD_B_DIO_CHANNELS[i],
                                            Calibrations.MAX_TURN_VELOCITY_RPS[i],
                                            Calibrations.MAX_TURN_ACCELERATION_RPSS[i],
                                            Calibrations.TURN_FF_KS_GAIN[i],
                                            Calibrations.TURN_FF_KV_GAIN[i],
                                            Calibrations.TURN_FF_KA_GAIN[i],
                                            Calibrations.DRIVE_FF_KS_GAIN[i],
                                            Calibrations.DRIVE_FF_KV_GAIN[i],
                                            Calibrations.DRIVE_FF_KA_GAIN[i]);
            m_desiredModulesState[i] = new SwerveModuleState(0.0, new Rotation2d());
            m_currentModulesState[i] = m_modules[i].getCurrentState();
        }
        m_kinematics = new SwerveDriveKinematics(Constants.Drivetrain.MODULE_LOCATIONS);
        m_imu = new ADIS16470();
        m_odometry = new SwerveDriveOdometry(m_kinematics, Rotation2d.fromDegrees(0.0));
        m_chassisSpeeds = new ChassisSpeeds(0.0, 0.0, 0.0);
        m_currentState = StateType.Idle;
        m_isFieldOriented = true;
        m_imuYawAngleRot2D = new Rotation2d();

        if (Constants.Drivetrain.ENABLE_LOGGING) {
            m_log = DataLogManager.getLog();
            m_stateLogEntry = new StringLogEntry(m_log, "Drivetrain State");
            m_fieldOrientedLogEntry = new BooleanLogEntry(m_log, "Is Field Oriented");
        } else {
            m_log = null;
            m_stateLogEntry = null;
            m_fieldOrientedLogEntry = null;
        }
    }


    /**
     * This method is called periodically by the command scheduler and is run before any of the commands are serviced.
     */
    @Override 
    public void periodic() {
        m_imuYawAngleRot2D =  Rotation2d.fromDegrees(m_imu.getAngle());
        for (int i = 0; i < Constants.Drivetrain.numModules; i++) {
            m_currentModulesState[i] = m_modules[i].getCurrentState();
        }
        m_odometry.update(m_imuYawAngleRot2D, m_currentModulesState);
        logModulesTelemetry();
        logTelemetry();
    }

}
