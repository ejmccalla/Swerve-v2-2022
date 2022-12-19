package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Drivetrain;
import frc.robot.subsystems.Drivetrain.StateType;

/**
 * Implements a command to collect data to calibrate the swerve turning motor feed-forwards.
 *
 * <p>{@link frc.robot.Calibrations#TURN_FF_KS_GAIN}
 * {@link frc.robot.Calibrations#TURN_FF_KV_GAIN}
 * {@link frc.robot.Calibrations#TURN_FF_KA_GAIN}
 */
public class CalibrateTurnFF extends CommandBase {

    private enum State {
        RampPositive,
        RampNegative,
        StepPositive,
        StepNegative
    }

    private Drivetrain m_drivetrain;
    /** Divide by 50 because there are 50 loops per 1 second with a 20ms control loop. */
    private static final double m_rampRateVpl = 0.25 / 50;
    private static final double m_maxRampVoltage = 10.0;
    private static final double m_stepVoltage = 6.0;
    private boolean m_isFinished;
    private State m_state;
    private double m_voltage;
    private int m_offVoltageCounter;
    private int m_onVoltageCounter;

    /**
     * Constructor for the calibrate tunning feed-forward command.
     *
     * @param drivetrain uses the drivetrain subsystem
     */
    public CalibrateTurnFF(Drivetrain drivetrain) {
        m_drivetrain = drivetrain;
        addRequirements(m_drivetrain);
    }

    /**
     * Update the drivetrain state to indicate the subsystem is calibrating the modules.
     * 
     * <p>This command should be scheduled as non-interruptible.
     */
    @Override
    public void initialize() {
        m_drivetrain.updateState(StateType.Calibrating);
        m_drivetrain.setIdleModules();
        m_isFinished = false;
        m_voltage = 0.0;
        m_offVoltageCounter = 50;
        m_onVoltageCounter = 50;
        m_state = State.RampPositive;
    }

    /**
     * Run the calibration routine on each of the drivetrain swerve modules.
     */
    @Override
    public void execute() {
        switch (m_state) {
            case RampPositive:
                m_voltage += m_rampRateVpl;
                if (m_voltage >= m_maxRampVoltage) {
                    m_state = State.RampNegative;
                    m_voltage = 0.0;
                }
                break;
                
            case RampNegative:
                if (m_offVoltageCounter == 0) {
                    m_voltage -= m_rampRateVpl;
                    if (m_voltage <= -m_maxRampVoltage) {
                        m_state = State.StepPositive;
                        m_voltage = 0.0;
                        m_offVoltageCounter = 50;
                    }
                } else {
                    --m_offVoltageCounter;
                }
                break;

            case StepPositive:
                if (m_offVoltageCounter == 0) {
                    m_voltage = m_stepVoltage;
                    if (m_onVoltageCounter == 0) {
                        m_state = State.StepNegative;
                        m_onVoltageCounter = 100;
                    } else {
                        --m_onVoltageCounter;
                    }
                } else {
                    --m_offVoltageCounter;
                }
                break;
            
            case StepNegative:
                if (m_offVoltageCounter == 0) {
                    m_voltage = -m_stepVoltage;
                    if (m_onVoltageCounter == 0) {
                        m_voltage = 0;
                        m_isFinished = true;
                    } else {
                        --m_onVoltageCounter;
                    }
                } else {
                    --m_offVoltageCounter;
                }
                break;
            
            default:
                m_voltage = 0;
                m_isFinished = true;
                break;
        }

        m_drivetrain.setModulesTurnVoltage(m_voltage);

    }

    /**
     * The command is complete when all the stages are complete.
     * 
     * <p>This command should be scheduled as non-interruptible.
     */
    @Override
    public boolean isFinished() {
        return m_isFinished;
    }

    /**
     * Update the drivetrain state when all of the modules have completed the Calibration.
     * 
     * <p>This command should be scheduled as non-interruptible.
     */
    @Override
    public void end(boolean interrupted) {
        m_drivetrain.setModulesIdleMode(false);
        m_drivetrain.setIdleModules();
        m_drivetrain.updateState(StateType.Idle);
    }

}
