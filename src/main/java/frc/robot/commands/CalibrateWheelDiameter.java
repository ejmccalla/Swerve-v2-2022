package frc.robot.commands;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Drivetrain;
import frc.robot.subsystems.Drivetrain.StateType;

/**
 * Implements a command to run to help with calibrating the wheel diameter.
 * {@link frc.robot.Calibrations#WHEEL_DIAMETER_INCH}.
 */
public class CalibrateWheelDiameter extends CommandBase {

    private Drivetrain m_drivetrain;

    /**
     * Constructor for the calibrate wheel diameter command.
     *
     * @param drivetrain uses the drivetrain for drive encoder feedback to the smart dashboard
     */
    public CalibrateWheelDiameter(Drivetrain drivetrain) {
        m_drivetrain = drivetrain;
        addRequirements(m_drivetrain);
    }

    /**
     * Set the swerve modules to coast mode (since the robot will manually be pushed) and reset the drive encoders to 0.
     */
    @Override
    public void initialize() {
        m_drivetrain.updateState(StateType.Calibrating);
        m_drivetrain.resetModulesDriveEncoder();
    }

    /**
     * Set the swerve modules motor ouptuts to 0 volts. The drive encoder positions are output to the smart dashboard for 
     * recording.
     */
    @Override
    public void execute() {
        m_drivetrain.setIdleModules();
        SmartDashboard.putNumberArray("Wheel Calibration Rotations", m_drivetrain.getModulesDriveRotations());
    }

    /**
     * Update the drivetrain state when the command is finished.
     */
    @Override
    public void end(boolean interrupted) {
        m_drivetrain.setModulesIdleMode(false);
        m_drivetrain.setIdleModules();
        m_drivetrain.updateState(StateType.Idle);

    }

}
