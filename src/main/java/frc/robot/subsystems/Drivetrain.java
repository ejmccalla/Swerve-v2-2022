package frc.robot.subsystems;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveDriveOdometry;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.util.datalog.DataLog;
import edu.wpi.first.util.datalog.StringLogEntry;
import edu.wpi.first.wpilibj.ADIS16470_IMU;
import edu.wpi.first.wpilibj.DataLogManager;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Calibrations;
import frc.robot.Constants;
import frc.robot.lib.SwerveDriveSignal;
import frc.robot.lib.SwerveModule;

/**
 * Implements the swerve drivetrain using REV/SparkMax, swerve MK4 modules, CTRE mag encoders, and
 * an ADIS16470 IMU.
 */
public class Drivetrain extends SubsystemBase {

    /**
     * The states of the drivetrain.
     *
     * <p>Homing will run the homing routine on all of the modules. Drive is will apply the drive
     * signal from the controller. PathFollowing will follow the given path trajectories.
     */
    private enum StateType {
        Homing { 
            @Override
            public String toString() {
                return "Homing";
            }
        },
        Drive {
            @Override
            public String toString() {
                return "Drive";
            }
        },
        PathFollowing {
            @Override
            public String toString() {
                return "Path Following";
            }
        },
        Idle {
            @Override
            public String toString() {
                return "Idle";
            }
        },
    }

    private final SwerveModule[] m_modules;
    private final SwerveDriveKinematics m_kinematics;
    private final ADIS16470_IMU m_imu;
    private final SwerveDriveOdometry m_odometry;
    private final DataLog m_log;
    private SwerveModuleState[] m_currentModulesState;
    private SwerveModuleState[] m_desiredModulesState;
    private ChassisSpeeds m_chassisSpeeds;
    private boolean m_areAllModulesHomed;
    private SwerveDriveSignal m_driveSignal;
    private boolean m_isFieldOriented;
    private StateType m_currentState;
    private StateType m_desiredState;
    private StringLogEntry m_stateLogEntry;


    //-------------------------------------------------------------------------------------------//
    /*                                      PUBLIC METHODS                                       */
    //-------------------------------------------------------------------------------------------//


    /**
     * Set whether the drive commands are field oriented or robot oriented.
     *
     * @param isFieldOriented true for field oriented drive, false for robot oriented drive
     */
    public void setIsFieldOriented(boolean isFieldOriented) {
        m_isFieldOriented = isFieldOriented;
    }

    /**
     * Request to drive the robot using controllers.
     *
     * <p>This will set the swerve drive signal and the desired state to <b>Drive</b>. This should
     * be called by a command.
     *
     * @param driveSignal translational speeds and rotational speed
     */
    public void requestDrive(SwerveDriveSignal driveSignal) {
        m_driveSignal = driveSignal;
        m_desiredState = StateType.Drive;
    }

    /**
     * Request to home the swerve modules.
     *
     * <p>This will set the desired state to <b>Homing</b>. This should be called by a command.
     */
    public void requestHoming() {
        m_desiredState = StateType.Homing;
    }

    /**
     * Request to set the all swerve motor output to 0 volts.
     *
     * <p>This will set the desired state to <b>Idle</b>. This should be called by a command.
     */
    public void requestIdle() {
        m_desiredState = StateType.Idle;
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
     * Get the drive encoder postion for all modules in their native units of rotations.
     *
     * @return the rotations of the swerve drive encoders
     */
    public double[] getModulesDriveRotations() {
        double[] rotations = new double[Constants.Drivetrain.numModules];
        for (int i = 0; i < Constants.Drivetrain.numModules; i++) {
            m_modules[i].getDriveEncPosRot();
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

    /**
     * This will set all of the module motors output to 0 volts.
     */
    private void setModulesToIdle() {
        for (int i = 0; i < Constants.Drivetrain.numModules; i++) {
            m_modules[i].setIdle();
        }
    }

    /**
     * Update the swerve odometry with the current swerve modules state.
     */
    private void updateOdometry() {
        for (int i = 0; i < Constants.Drivetrain.numModules; i++) {
            m_currentModulesState[i] = m_modules[i].getCurrentState();
        }
        m_odometry.update(getImuYawAngleRot(), m_currentModulesState);
    }

    /**
     * Get the yaw angle of the IMU.
     *
     * @return the IMU yaw angle as a Rotation2D object
     */
    private Rotation2d getImuYawAngleRot() {
        return Rotation2d.fromDegrees(m_imu.getAngle());
    }

    /** 
     * Set the state of the swerve modules to perform the desired auto/teleop command.
     */
    private void setDesiredModulesState() {
        if (m_isFieldOriented) {
            m_chassisSpeeds = 
                ChassisSpeeds.fromFieldRelativeSpeeds(m_driveSignal.getxSpeed(),
                                                      m_driveSignal.getySpeed(),
                                                      m_driveSignal.getRotationalSpeed(),
                                                      getImuYawAngleRot());
        } else {
            m_chassisSpeeds = new ChassisSpeeds(m_driveSignal.getxSpeed(),
                                                m_driveSignal.getySpeed(),
                                                m_driveSignal.getRotationalSpeed());
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
     * Set the state of the swerve modules to turn to their home position.
     */
    private void setHomedModulesState() {
        m_areAllModulesHomed = true;
        for (int i = 0; i < Constants.Drivetrain.numModules; i++) {
            m_modules[i].setHomedModuleState();
            m_areAllModulesHomed = m_areAllModulesHomed && m_modules[i].getIsHomed();
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
        m_imu = new ADIS16470_IMU();
        m_odometry = new SwerveDriveOdometry(m_kinematics, Rotation2d.fromDegrees(0.0));
        m_chassisSpeeds = new ChassisSpeeds(0.0, 0.0, 0.0);
        m_currentState = StateType.Homing;
        m_desiredState = StateType.Idle;
        m_areAllModulesHomed = false;
        m_driveSignal = new SwerveDriveSignal(0.0, 0.0, 0.0);
        m_isFieldOriented = true;

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
     *
     * <p>First, the telemetry (for both the drivetrain and the swerve modules) is logged and the
     * odometry is updated. Next, the state machine will update. This includes updating the current
     * state variable and setting the swerve module states or motor outputs. Finally, the desired
     * state variable is reset to the <b>Idle</b> state. It is expected that one of the drivetrain
     * commandswill overwrite this with one of the other states which does something.
     *
     * <p>The <b>Idle</b> state is the default state when no other states have been requested. In
     * this state the swerve modules turning and driving motor outputs are set to 0 volts. The
     * current state is updated to the desired state.
     *
     * <p>The <b>Homing</b> state will call the homing routine for each of the swerve modules and
     * then check to see if they're homed. If they're homed, the current state is updated to the
     * desired state. If they're not homed, the current state is left unchanged. This state should
     * only ever be set when the robot is initially powered on. This is the only state which can
     * block the current state being updated to the desired state. As such, it is important to
     * ensure that the module homing routine is robust.
     *
     * <p>The <b>Drive</b> state will take the drive signal, convert it to the desired swerve
     * module state, and apply the states to the swerve modules. The current state is updated to
     * the desired state.
     *
     * <p>The <b>PathFollowing</b> state will TODO: add autonomous.
     *
     * <p>The <b>default</b> state, which should never happen, will apply 0 volts to the swerve
     * modules turning and driving motor outputs. The current state is updated to
     * the desired state.
     */
    @Override 
    public void periodic() {

        logModulesTelemetry();
        logTelemetry();
        updateOdometry();

        StateType nextState = m_currentState;
        switch (m_currentState) {
            case Idle:
                setModulesToIdle();
                nextState = m_desiredState;
                break;

            case Homing:
                m_areAllModulesHomed = true;
                setHomedModulesState();
                //TODO: Add a timeout to the homing routine.
                if (m_areAllModulesHomed) {
                    nextState = m_desiredState;
                }
                break;

            case Drive:
                setDesiredModulesState();
                nextState = m_desiredState;
                break;

            default:
                setModulesToIdle();
                nextState = StateType.Idle;
        }
        m_currentState = nextState;
        m_desiredState = StateType.Idle;
    }

}
