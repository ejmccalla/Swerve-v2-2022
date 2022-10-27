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
    private StateType m_currentState;
    private StateType m_desiredState;
    private StringLogEntry m_stateLogEntry;


    //-------------------------------------------------------------------------------------------//
    /*                                      PUBLIC METHODS                                       */
    //-------------------------------------------------------------------------------------------//

    /**
     * Get the yaw angle of the IMU.
     *
     * @return the IMU yaw angle in degrees
     */
    public double getImuYawAngleDeg() {
        return m_imu.getAngle();
    }

    /**
     * Drive the robot using controllers.
     *
     * <p>This will set the swerve drive signal and the desired state to <b>Drive</b>. This should
     * be called by a command (which will likely be the default subsystem command).
     *
     * @param driveSignal has all three speeds and a field relative flag
     */
    public void drive(SwerveDriveSignal driveSignal) {
        m_driveSignal = driveSignal;
        m_desiredState = StateType.Drive;
    }


    //-------------------------------------------------------------------------------------------//
    /*                                     PRIVATE METHODS                                       */
    //-------------------------------------------------------------------------------------------//


    /**
     * Get the yaw angle of the IMU.
     *
     * @return the IMU yaw angle as a Rotation2D object
     */
    private Rotation2d getImuYawAngleRot() {
        return Rotation2d.fromDegrees(m_imu.getAngle());
    }

    /** 
     * Set the state of the swerve modules.
     */
    private void setDesiredModulesState() {
        // TODO: Calibrate the velocity constraint.
        SwerveDriveKinematics.desaturateWheelSpeeds(m_desiredModulesState, 
                                                    Calibrations.MAX_DRIVE_VELOCITY_MPS);
        for (int i = 0; i < Constants.Drivetrain.numModules; i++) {
            m_modules[i].setDesiredState(m_desiredModulesState[i]);
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
        }
        m_kinematics = new SwerveDriveKinematics(Constants.Drivetrain.MODULE_LOCATIONS);
        m_imu = new ADIS16470_IMU();
        m_odometry = new SwerveDriveOdometry(m_kinematics, Rotation2d.fromDegrees(0.0));
        m_chassisSpeeds = new ChassisSpeeds(0.0, 0.0, 0.0);
        m_currentState = StateType.Homing;
        m_desiredState = StateType.Homing;
        m_areAllModulesHomed = false;
        m_driveSignal = new SwerveDriveSignal(0.0, 0.0, 0.0, true);

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
     * odometry is updated. Next, the state machine will update. This includes updating the next
     * state, as well as, updating the swerve module states.
     *
     * <p>The <b>Homing</b> state will call the homeModule routine for each of the swerve modules
     * and then check to see if they're homed. If they are, the state machine will update to the
     * desired state (either teleop Drive or auto PathFollowing).
     *
     * <p>The <b>Drive</b> state will take the drive signal, convert it to the desired swerve
     * module state, and apply the states to the swerve modules.
     *
     * <p>The <b>PathFollowing</b> state will take the auto trajectory, convert it to the desired
     * swerve module state, and apply the states to the swerve modules.
     *
     * <p>The <b>default</b> state, which should never happen, will apply no drive velocity and
     * maintain the current angle of the swerve modules.
     */
    @Override 
    public void periodic() {

        for (int i = 0; i < Constants.Drivetrain.numModules; i++) {
            m_modules[i].logTelemetry();
            m_currentModulesState[i] = m_modules[i].getCurrentState();
        }
        m_odometry.update(getImuYawAngleRot(), m_currentModulesState);

        if (Constants.Drivetrain.ENABLE_LOGGING) {
            m_stateLogEntry.append(m_currentState.toString());
        }

        StateType nextState = m_currentState;
        switch (m_currentState) {
            case Homing:
                m_areAllModulesHomed = true;
                for (int i = 0; i < Constants.Drivetrain.numModules; i++) {
                    m_modules[i].homeModule();
                    m_areAllModulesHomed = m_areAllModulesHomed && m_modules[i].getIsHomed();
                }
                //TODO: Add a timeout to the homing routine.
                if (m_areAllModulesHomed) {
                    nextState = m_desiredState;
                }
                break;
            
            case Drive:
                if (m_driveSignal.getIsFieldOriented()) {
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
                setDesiredModulesState();
                nextState = m_desiredState;
                break;

            default:
                for (int i = 0; i < Constants.Drivetrain.numModules; i++) {
                    m_desiredModulesState[i] = m_modules[i].getCurrentState();
                    m_desiredModulesState[i].speedMetersPerSecond = 0.0;
                }
                setDesiredModulesState();
        }
        m_currentState = nextState;
    }

}
