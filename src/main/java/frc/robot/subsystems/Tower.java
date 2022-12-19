package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkMaxPIDController;
import edu.wpi.first.util.datalog.DataLog;
import edu.wpi.first.util.datalog.DoubleLogEntry;
import edu.wpi.first.util.datalog.StringLogEntry;
import edu.wpi.first.wpilibj.DataLogManager;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Calibrations;
import frc.robot.Constants;

/**
 * Implements the tower with a NEO 550 and 5:1 reduction.
 */
public class Tower extends SubsystemBase {

    /**
     * The states of the tower.
     * 
     * <p>Commands are responsible for setting the state of the tower and are defined as follows:
     *
     * <p><b>Idle</b> - there are no commands currently using the subsystem.
     *
     * <p><b>LoadCargo</b> - the tower pushing cargo into the shooter.
     *
     * <p><b>UnloadCargo</b> - the tower pulling cargo out of the shooter.
     */
    public enum StateType {
        Idle { 
            @Override
            public String toString() {
                return "Idle";
            }
        },
        LoadCargo { 
            @Override
            public String toString() {
                return "Load Cargo";
            }
        },
        UnloadCargo {
            @Override
            public String toString() {
                return "Unload Cargo";
            }
        },
    }

    private final CANSparkMax m_motor;
    private SparkMaxPIDController m_pidController;
    private RelativeEncoder m_encoder;
    private final DataLog m_log;
    private StateType m_currentState;
    private DoubleLogEntry m_wheelsLinearVelocityMps;
    private StringLogEntry m_stateLogEntry;


    //--------------------------------------------------------------------------------------------------------------------//
    /*                                                   PUBLIC METHODS                                                   */
    //--------------------------------------------------------------------------------------------------------------------//


    /**
     * Move the cargo into the shooter.
     */
    public void loadCargo() {
        m_currentState = StateType.LoadCargo;
        m_pidController.setReference(Calibrations.Tower.loadTargetRpm, CANSparkMax.ControlType.kVelocity);
    }

    /**
     * Pull the cargo out of the shooter.
     */
    public void unloadCargo() {
        m_currentState = StateType.UnloadCargo;
        m_pidController.setReference(-Calibrations.Tower.unloadTargetRpm, CANSparkMax.ControlType.kVelocity);
    }

    /**
     * Turn off the motor output.
     */
    public void turnOffTower() {
        m_currentState = StateType.Idle;
        m_pidController.setReference(0.0, CANSparkMax.ControlType.kDutyCycle);
    }


    //--------------------------------------------------------------------------------------------------------------------//
    /*                                                  PRIVATE METHODS                                                   */
    //--------------------------------------------------------------------------------------------------------------------//


    /**
     * Log the telemetry data to disk using the WPILib logger.
     */
    private void logTelemetry() {
        if (Constants.Tower.ENABLE_LOGGING) {
            m_wheelsLinearVelocityMps.append(m_encoder.getVelocity());
            m_stateLogEntry.append(m_currentState.toString());
        }
    }


    //--------------------------------------------------------------------------------------------------------------------//
    /*                                         CONSTRUCTOR AND PERIODIC METHODS                                           */
    //--------------------------------------------------------------------------------------------------------------------//


    /** 
     * Constructor for the tower.
     *
     * <p>The NEO 550 motor is fragile when it comes to higher current draws which are common when a motor is stalled. For
     * the tower, stalling the motor is possible unless synced up with the shooter. With this in mind, it is important to
     * set safe current limits and fix any "lack of power" issues with gearing, a bigger motor, or adding another motor.
     *
     * @see <a href="https://www.revrobotics.com/neo-550-brushless-motor-locked-rotor-testing/">NEO 550 Locked Rotor testing</a>
     */
    public Tower() {
        m_motor = new CANSparkMax(Constants.Tower.MOTOR_ID, MotorType.kBrushless);
        m_motor.restoreFactoryDefaults();
        m_motor.setSmartCurrentLimit(20);
        m_motor.enableVoltageCompensation(12.0);
        m_motor.setIdleMode(IdleMode.kCoast);

        m_pidController = m_motor.getPIDController();
        m_pidController.setP(Calibrations.Tower.P);
        m_pidController.setI(0);
        m_pidController.setD(Calibrations.Tower.D);
        m_pidController.setIZone(0);
        m_pidController.setFF(Calibrations.Tower.FF);
        m_pidController.setOutputRange(-1, 1);

        m_encoder = m_motor.getEncoder();
        m_encoder.setVelocityConversionFactor(Constants.Tower.WHEEL_DIAMETER_FT * Math.PI
            / Constants.Tower.GEAR_RATIO / 60.0);

        //m_motor.burnFlash();

        m_currentState = StateType.Idle;

        if (Constants.Tower.ENABLE_LOGGING) {
            m_log = DataLogManager.getLog();
            m_wheelsLinearVelocityMps = new DoubleLogEntry(m_log, "Tower Wheels Linear Velocity (fps)");
            m_stateLogEntry = new StringLogEntry(m_log, "Tower State");
        } else {
            m_log = null;
            m_wheelsLinearVelocityMps = null;
            m_stateLogEntry = null;
        }
    }


    /**
     * This method is called periodically by the command scheduler and is run before any of the commands are serviced.
     */
    @Override 
    public void periodic() {
        logTelemetry();
    }

}
