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
 * Implementation of a swerve drivetrain using REV/SparkMax.
 */
public class Drivetrain extends SubsystemBase {

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
    private SwerveModuleState[] m_currentModuleStates;
    private SwerveModuleState[] m_desiredModuleStates;
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
     * Get the heading of the robot.
     *
     * @return the IMU yaw angle in degrees.
     */
    public double getImuYawAngleDeg() {
        return m_imu.getAngle();
    }

    /**
     * Method to drive the robot using joystick info. This will set the swerve drive signal and
     * set the desired state to drive (teleop).
     *
     * @param driveSignal has all three speeds and a field relative flag.
     */
    public void drive(SwerveDriveSignal driveSignal) {
        m_driveSignal = driveSignal;
        m_desiredState = StateType.Drive;
    }


    //-------------------------------------------------------------------------------------------//
    /*                                     PRIVATE METHODS                                       */
    //-------------------------------------------------------------------------------------------//


    /** Returns the IMU yaw angle as a Rotation2d object. */
    private Rotation2d getImuRotation() {
        return Rotation2d.fromDegrees(m_imu.getAngle());
    }

    /** Set the swerve module states. */
    private void setDesiredModuleStates() {
        SwerveDriveKinematics.desaturateWheelSpeeds(m_desiredModuleStates, 
                                                    Calibrations.MAX_DRIVE_VELOCITY_MPS);
        for (int i = 0; i < Constants.Drivetrain.numModules; i++) {
            m_modules[i].setState(m_desiredModuleStates[i]);
        }
    }


    //-------------------------------------------------------------------------------------------//
    /*                            CONSTRUCTOR AND PERIODIC METHODS                               */
    //-------------------------------------------------------------------------------------------//


    /** Constructor for the drivetrain. */
    public Drivetrain() {
        m_modules = new SwerveModule[Constants.Drivetrain.numModules];
        m_desiredModuleStates = new SwerveModuleState[Constants.Drivetrain.numModules];
        m_currentModuleStates = new SwerveModuleState[Constants.Drivetrain.numModules];
        for (int i = 0; i < Constants.Drivetrain.numModules; i++) {
            m_modules[i] = new SwerveModule(Constants.Drivetrain.MODULE_LABELS[i],
                                            Calibrations.ZEROS_RAD[i],
                                            Constants.Drivetrain.TURN_IDS[i],
                                            Constants.Drivetrain.DRIVE_IDS[i],
                                            Constants.Drivetrain.PWM_DIO_CHANNELS[i],
                                            Constants.Drivetrain.QUAD_A_DIO_CHANNELS[i],
                                            Constants.Drivetrain.QUAD_B_DIO_CHANNELS[i],
                                            0.02);
            m_desiredModuleStates[i] = new SwerveModuleState(0.0, new Rotation2d());
            m_currentModuleStates[i] = m_modules[i].getState();
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


    /** This method is called periodically before any commands. */
    @Override 
    public void periodic() {

        for (int i = 0; i < Constants.Drivetrain.numModules; i++) {
            m_modules[i].logTelemetry();
            m_currentModuleStates[i] = m_modules[i].getState();
        }
        m_odometry.update(getImuRotation(), m_currentModuleStates);

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
                                                              getImuRotation());
                } else {
                    m_chassisSpeeds = new ChassisSpeeds(m_driveSignal.getxSpeed(),
                                                        m_driveSignal.getySpeed(),
                                                        m_driveSignal.getRotationalSpeed());
                }
                m_desiredModuleStates = m_kinematics.toSwerveModuleStates(m_chassisSpeeds);
                setDesiredModuleStates();
                nextState = m_desiredState;
                break;

            default:
                for (int i = 0; i < Constants.Drivetrain.numModules; i++) {
                    m_desiredModuleStates[i] = m_modules[i].getState();
                    m_desiredModuleStates[i].speedMetersPerSecond = 0.0;
                }
                setDesiredModuleStates();
        }
        m_currentState = nextState;
    }

}
