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
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.Solenoid;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Calibrations;
import frc.robot.Constants;

/**
 * Implements the intake with a NEO 550 and 3:1 reduction.
 */
public class Intake extends SubsystemBase {

    /**
     * The states of the intake.
     * 
     * <p>Commands are responsible for setting the state of the tower and are defined as follows:
     *
     * <p><b>Idle</b> - There are no commands currently using the subsystem.
     *
     * <p><b>Retracted</b> - the intake is retracted the motor is spun down.
     *
     * <p><b>Extended</b> - The intake is extended and the motor is spun up.
     */
    public enum StateType {
        Idle { 
            @Override
            public String toString() {
                return "Idle";
            }
        },
        Retracted { 
            @Override
            public String toString() {
                return "Retracted";
            }
        },
        Extended {
            @Override
            public String toString() {
                return "Extended";
            }
        },
    }

    private final CANSparkMax m_motor;
    private SparkMaxPIDController m_pidController;
    private RelativeEncoder m_encoder;
    private final Solenoid m_solenoid;
    private final DataLog m_log;
    private StateType m_currentState;
    private DoubleLogEntry m_rollerLinearVelocityMps;
    private StringLogEntry m_stateLogEntry;


    //--------------------------------------------------------------------------------------------------------------------//
    /*                                                   PUBLIC METHODS                                                   */
    //--------------------------------------------------------------------------------------------------------------------//


    /**
     * Toggle the intake between extended and retracted.
     * 
     * <p>It is expected that this will be used as an instant command to extend and retract the intake. 
     */
    public void toggleIntake() {
        if (m_solenoid.get()) {
            retractIntake();
        } else {
            extendIntake();
        }
    }


    /**
     * Extend the intake and start the motor.
     */
    public void extendIntake() {
        m_currentState = StateType.Extended;
        m_solenoid.set(true);
        m_pidController.setReference(Calibrations.Intake.extendedTargetRpm, CANSparkMax.ControlType.kVelocity);
    }


    /**
     * Retract the intake and stop the motor.
     */
    public void retractIntake() {
        m_currentState = StateType.Retracted;
        m_solenoid.set(false);
        m_pidController.setReference(Calibrations.Intake.retractedTargetRpm, CANSparkMax.ControlType.kVelocity);
    }


    //--------------------------------------------------------------------------------------------------------------------//
    /*                                                  PRIVATE METHODS                                                   */
    //--------------------------------------------------------------------------------------------------------------------//


    /**
     * Log the telemetry data to disk using the WPILib logger.
     */
    private void logTelemetry() {
        if (Constants.Intake.ENABLE_LOGGING) {
            m_rollerLinearVelocityMps.append(m_encoder.getVelocity());
            m_stateLogEntry.append(m_currentState.toString());
        }
    }


    //--------------------------------------------------------------------------------------------------------------------//
    /*                                         CONSTRUCTOR AND PERIODIC METHODS                                           */
    //--------------------------------------------------------------------------------------------------------------------//


    /** 
     * Constructor for the intake.
     *
     * <p>The NEO 550 motor is fragile when it comes to higher current draws which are common when a motor is stalled. For
     * an intake, stalling the motor can be a fairly common occurrence. This is especially true when working through the
     * early iterations of a design. With this in mind, it is important to set safe current limits and fix any "lack of
     * power" issues with gearing, a bigger motor, or adding another motor.
     *
     * <p>The intake is actuated using a single-acting solenoid. This meaning that it is plumbed to default (power off
     * state) to the intake being up.
     *
     * @see <a href="https://www.revrobotics.com/neo-550-brushless-motor-locked-rotor-testing/">NEO 550 Locked Rotor testing</a>
     */
    public Intake() {
        m_motor = new CANSparkMax(Constants.Intake.MOTOR_ID, MotorType.kBrushless);
        m_motor.restoreFactoryDefaults();
        m_motor.setSmartCurrentLimit(20);
        m_motor.enableVoltageCompensation(12.0);
        m_motor.setIdleMode(IdleMode.kCoast);

        m_pidController = m_motor.getPIDController();
        m_pidController.setP(Calibrations.Intake.P);
        m_pidController.setI(0);
        m_pidController.setD(Calibrations.Intake.D);
        m_pidController.setIZone(0);
        m_pidController.setFF(Calibrations.Intake.FF);
        m_pidController.setOutputRange(-1, 1);

        m_encoder = m_motor.getEncoder();
        m_encoder.setVelocityConversionFactor(Constants.Intake.ROLLER_DIAMETER_FT * Math.PI
            / Constants.Intake.GEAR_RATIO / 60.0);

        //m_motor.burnFlash();

        m_solenoid = new Solenoid(Constants.Hardware.REV_PH_ID, PneumaticsModuleType.REVPH, Constants.Intake.SOLENOID_ID);
        m_solenoid.set(false);

        m_currentState = StateType.Idle;

        if (Constants.Intake.ENABLE_LOGGING) {
            m_log = DataLogManager.getLog();
            m_rollerLinearVelocityMps = new DoubleLogEntry(m_log, "Intake Roller Linear Velocity (fps)");
            m_stateLogEntry = new StringLogEntry(m_log, "Intake State");
        } else {
            m_log = null;
            m_rollerLinearVelocityMps = null;
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
