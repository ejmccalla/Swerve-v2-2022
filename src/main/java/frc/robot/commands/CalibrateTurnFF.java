package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Drivetrain;
import frc.robot.subsystems.Drivetrain.StateType;

/**
 * Implements a command to home all of the swerve modules.
 */
public class CalibrateTurnFF extends CommandBase {

    private Drivetrain m_drivetrain;

    /** Devide by 50 because there are 50 loops per 1 second with a 20ms control loop. */
    private static final double m_rampRateVpl = 0.25 / 50;
    private static final double m_maxRampVoltage = 4.0;
    private static final double m_stepVoltage = 4.0;

    private boolean m_isFinished;

    private enum State {
        RampPositive,
        RampNegitive,
        StepPositive,
        StepNegitive
    }

    private State m_state;
    private double m_voltage;
    private int m_offVoltageCounter;
    private int m_onVoltageCounter;

    /**
     * Contructor.
     *
     * @param drivetrain this commands needs the drivetrain
     */
    public CalibrateTurnFF(Drivetrain drivetrain) {
        m_drivetrain = drivetrain;
        addRequirements(m_drivetrain);
    }

    /**
     * Update the drivetrain state to indicate the subysystem is calibrating the modules.
     * 
     * <p>This command should be scheduled as non-interruptable.
     */
    @Override
    public void initialize() {
        m_drivetrain.updateState(StateType.Calibrating);
        m_drivetrain.setIdleModules();
        m_isFinished = false;
        m_voltage = 0.0;
        m_offVoltageCounter = 50;
        m_onVoltageCounter = 50;
        m_drivetrain.setModulesToBrakeMode(true);
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
                    m_state = State.RampNegitive;
                    m_voltage = 0.0;
                }
                break;
                
            case RampNegitive:
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
                        m_state = State.StepNegitive;
                        m_onVoltageCounter = 100;
                    } else {
                        --m_onVoltageCounter;
                    }
                } else {
                    --m_offVoltageCounter;
                }
                break;
            
            case StepNegitive:
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
     * <p>This command should be scheduled as non-interruptable.
     */
    @Override
    public boolean isFinished() {
        return m_isFinished;
    }

    /**
     * Update the drivetrain state when all of the modules have completed the Calibration.
     * 
     * <p>This command should be scheduled as non-interruptable.
     */
    @Override
    public void end(boolean interrupted) {
        m_drivetrain.setModulesToBrakeMode(false);
        m_drivetrain.setIdleModules();
        m_drivetrain.updateState(StateType.Idle);
    }

}
