package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Drivetrain;
import frc.robot.subsystems.Drivetrain.StateType;

/**
 * Implements a command to home all of the swerve modules.
 */
public class HomeSwerveModules extends CommandBase {

    private Drivetrain m_drivetrain;

    /**
     * Contructor.
     *
     * @param drivetrain this commands needs the drivetrain
     */
    public HomeSwerveModules(Drivetrain drivetrain) {
        m_drivetrain = drivetrain;
        addRequirements(m_drivetrain);
    }

    /**
     * Update the drivetrain state to indicate the subysystem is homing the modules.
     * 
     * <p>This command should be scheduled as non-interruptable.
     */
    @Override
    public void initialize() {
        m_drivetrain.updateState(StateType.Homing);
        m_drivetrain.configureModulesTurningController(false);
    }

    /**
     * Run the homing routine on each of the drivetrain swerve modules.
     */
    @Override
    public void execute() {
        m_drivetrain.setHomedModulesState();
    }

    /**
     * The command is complete when all modules are homed.
     * 
     * <p>This command should be scheduled as non-interruptable.
     * 
     * <p>TODO: Add a timeout to the homing routine, but then what to do??
     */
    @Override
    public boolean isFinished() {
        return m_drivetrain.getAreAllModulesHomed();
    }

    /**
     * Update the drivetrain state when all of the modules have successfully completed the homing
     * sequence.
     * 
     * <p>This command should be scheduled as non-interruptable.
     */
    @Override
    public void end(boolean interrupted) {
        m_drivetrain.updateState(StateType.Idle);
        m_drivetrain.configureModulesTurningController(true);
    }

}
