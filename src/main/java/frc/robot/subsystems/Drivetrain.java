package frc.robot.subsystems;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveDriveOdometry;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.util.datalog.DataLog;
import edu.wpi.first.util.datalog.StringLogEntry;
// import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.DataLogManager;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Calibrations;
import frc.robot.Constants;
import frc.robot.lib.SwerveDriveSignal;
import frc.robot.lib.SwerveModule;
import frc.robot.lib.ADIS16470;

/**
 * Implements the swerve drivetrain using REV/SparkMax, swerve MK4 modules, CTRE mag encoders, and
 * an ADIS16470 IMU.
 */
public class Drivetrain extends SubsystemBase {

    /**
     * The states of the drivetrain.
     *
     * <p>Commands are responsible for setting the state of the drivetrain and are defined as
     * follows:
     *
     * <p><b>Idle</b> - There are no commands currently using the subsystem.
     *
     * <p><b>Homing</b> - The swerve modules are homing.
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
        Homing { 
            @Override
            public String toString() {
                return "Homing";
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
    private boolean m_areAllModulesHomed;
    private boolean m_isFieldOriented;
    private Rotation2d m_imuYawAngleRot2D;
    private StateType m_currentState;
    private StringLogEntry m_stateLogEntry;


    //-------------------------------------------------------------------------------------------//
    /*                                      PUBLIC METHODS                                       */
    //-------------------------------------------------------------------------------------------//


    /**
     * Set all of the swerve module motor outputs to 0 volts.
     */
    public void setIdleModules() {
        for (int i = 0; i < Constants.Drivetrain.numModules; i++) {
            m_modules[i].setIdle();
        }
    }


    /**
     * Set the state of the swerve modules to turn to their home position.
     */
    public void setHomedModulesState() {
        m_areAllModulesHomed = true;
        for (int i = 0; i < Constants.Drivetrain.numModules; i++) {
            m_modules[i].setHomedModuleState();
            m_areAllModulesHomed = m_areAllModulesHomed && m_modules[i].getIsHomed();
        }
    }

    /**
     * Sets the commanded voltage of the turn motors.
     * 
     * @param voltage the voltage the motors is set to.
     */
    public void setModulesTurnVoltage(double voltage) {
        for (int i = 0; i < Constants.Drivetrain.numModules; i++) {
            m_modules[i].setTurnVoltage(voltage);
        }
    }

    /**
     * Reset the profiled PID turning controllers which will zero out the integral term and
     * update the setpoint to the current angle of the absolute encoder.
     */
    public void resetModulesTurningController() {
        for (int i = 0; i < Constants.Drivetrain.numModules; i++) {
            m_modules[i].resetTurningController();
        }
    }


    /** 
     * Set the state of the swerve modules to perform the desired auto/teleop command.
     */
    public void setDesiredModulesState(SwerveDriveSignal driveSignal) {
        if (m_isFieldOriented) {
            m_chassisSpeeds = 
                ChassisSpeeds.fromFieldRelativeSpeeds(driveSignal.getxSpeed(),
                                                      driveSignal.getySpeed(),
                                                      driveSignal.getRotationalSpeed(),
                                                      m_imuYawAngleRot2D);
        } else {
            m_chassisSpeeds = new ChassisSpeeds(driveSignal.getxSpeed(),
                                                driveSignal.getySpeed(),
                                                driveSignal.getRotationalSpeed());
        }
        m_desiredModulesState = m_kinematics.toSwerveModuleStates(m_chassisSpeeds);

        // TODO: Calibrate the velocity constraint.
        SwerveDriveKinematics.desaturateWheelSpeeds(m_desiredModulesState, 
                                                    Calibrations.MAX_DRIVE_VELOCITY_MPS);
        for (int i = 0; i < Constants.Drivetrain.numModules; i++) {
            m_modules[i].setDesiredState(m_desiredModulesState[i]);
        }
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
     * Check if all of the swerve modules have completed the homing sequence.
     *
     * @return true if all the modules are homed, otherwise false
     */
    public boolean getAreAllModulesHomed() {
        return m_areAllModulesHomed;
    }

    /**
     * Set whether the drive commands are field oriented or robot oriented.
     *
     * @param isFieldOriented true for field oriented drive, false for robot oriented drive
     */
    public void setIsFieldOriented(boolean isFieldOriented) {
        m_isFieldOriented = isFieldOriented;
    }

    /**
     * Set the idle mode of the modules motors.
     *
     * @param isBrakeDesired true sets brake mode, false sets coast mode
     */
    public void setModulesToBrakeMode(boolean isBrakeDesired) {
        for (int i = 0; i < Constants.Drivetrain.numModules; i++) {
            m_modules[i].setModulesToBrakeMode(isBrakeDesired);
        }
    }

    /**
     * Set the drive encoder postion to 0 for all modules.
     */
    public void resetModulesDriveEncoder() {
        for (int i = 0; i < Constants.Drivetrain.numModules; i++) {
            m_modules[i].resetDriveEncoder();
        }
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
     * @return the IMU die temp in degrees celcius
     */
    public double getImuTempDegC() {
        return m_imu.getTemp();
    }

    /**
     * Get the drive encoder postion for all modules in their native units of rotations.
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


    //-------------------------------------------------------------------------------------------//
    /*                                     PRIVATE METHODS                                       */
    //-------------------------------------------------------------------------------------------//


    /**
     * Log the telemetry data to disk using the WPILib logger.
     */
    private void logTelemetry() {
        if (Constants.Drivetrain.ENABLE_LOGGING) {
            m_stateLogEntry.append(m_currentState.toString());
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

    //-------------------------------------------------------------------------------------------//
    /*                            CONSTRUCTOR AND PERIODIC METHODS                               */
    //-------------------------------------------------------------------------------------------//


    /** 
     * Constructor for the drivetrain.
     */
    public Drivetrain() {
        m_modules = new SwerveModule[Constants.Drivetrain.numModules];
        m_desiredModulesState = new SwerveModuleState[Constants.Drivetrain.numModules];
        m_currentModulesState = new SwerveModuleState[Constants.Drivetrain.numModules];
        for (int i = 0; i < Constants.Drivetrain.numModules; i++) {
            m_modules[i] = new SwerveModule(Constants.Drivetrain.MODULE_LABELS[i],
                                            Calibrations.ZEROS_RAD[i],
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
            m_modules[i].setModulesToBrakeMode(false);
        }
        m_kinematics = new SwerveDriveKinematics(Constants.Drivetrain.MODULE_LOCATIONS);
        m_imu = new ADIS16470();
        m_odometry = new SwerveDriveOdometry(m_kinematics, Rotation2d.fromDegrees(0.0));
        m_chassisSpeeds = new ChassisSpeeds(0.0, 0.0, 0.0);
        m_currentState = StateType.Idle;
        m_areAllModulesHomed = false;
        m_isFieldOriented = true;
        m_imuYawAngleRot2D = new Rotation2d();

        if (Constants.Drivetrain.ENABLE_LOGGING) {
            m_log = DataLogManager.getLog();
            m_stateLogEntry = new StringLogEntry(m_log, "Drivetrain State");
        } else {
            m_log = null;
            m_stateLogEntry = null;
        }
    }


    /**
     * This method is called periodically by the command scheduler and is run before any of the
     * commands are serviced.
     */
    @Override 
    public void periodic() {
        m_imuYawAngleRot2D =  Rotation2d.fromDegrees(m_imu.getAngle());
        for (int i = 0; i < Constants.Drivetrain.numModules; i++) {
            m_currentModulesState[i] = m_modules[i].getCurrentState();
            //SmartDashboard.putNumber("Abs Encoder "+i, m_currentModulesState[i].angle.getRadians());
        }
        m_odometry.update(m_imuYawAngleRot2D, m_currentModulesState);
        logModulesTelemetry();
        logTelemetry();
    }

}
