package frc.robot;

import edu.wpi.first.util.datalog.DataLog;
import edu.wpi.first.util.datalog.DoubleLogEntry;
import edu.wpi.first.wpilibj.Compressor;
import edu.wpi.first.wpilibj.DataLogManager;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.livewindow.LiveWindow;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandScheduler;

/**
 * This is the top-level class where the TimedRobot states are defined.
 *
 * <p>Use the WPILib data logger to log telemetry and other useful data for analysis and tuning.
 * Be sure to always use a USB thumb drive for logging. To do this, be sure to format the thumb
 * drive with a RoboRIO compatible FS (like FAT32) and simply plug into one of the two USB ports on
 * the RoboRIO.
 *
 * <p>The REV Robotics pressure sensor can be used for closed-loop control per the 2022 FRC game
 * rules.
 *
 * @see edu.wpi.first.wpilibj.IterativeRobotBase#loopFunc()
 * @see <a href="https://docs.wpilib.org/en/stable/docs/software/telemetry/datalog.html">WPILib Logger</a>
 * @see <a href="https://www.revrobotics.com/content/docs/REV-11-1107-DS.pdf">REV Pressure Sensor</a>
 */
public class Robot extends TimedRobot {

    /** This has the effect of a C/C++ precompile directive to enable/disable logging. */
    private static final boolean m_enableLogger = false;

    private RobotContainer m_robotContainer;
    private Compressor m_compressor;
    private double m_pressure;
    private DoubleLogEntry m_pressureLogEntry;
    private double m_compressorCurrent;
    private DoubleLogEntry m_compressorCurrentLogEntry;

    @Override
    public void robotInit() {
        m_robotContainer = new RobotContainer();
        if (m_enableLogger) {
            DataLogManager.start();
            DataLogManager.logNetworkTables(false);
            DataLog log = DataLogManager.getLog();
            m_pressureLogEntry = new DoubleLogEntry(log, "/pneumatics/pressure_sensor_PSI");
            m_compressorCurrentLogEntry = 
                new DoubleLogEntry(log, "/pneumatics/compressor_current_A");
        }
        m_compressor = new Compressor(Constants.Hardware.PCM_ID, PneumaticsModuleType.REVPH);
        m_compressor.disable();
        LiveWindow.disableAllTelemetry();
    }

    @Override
    public void robotPeriodic() {
        m_compressorCurrent = m_compressor.getCurrent();
        m_pressure = m_compressor.getPressure();
        if (m_enableLogger) {
            m_compressorCurrentLogEntry.append(m_compressorCurrent);
            m_pressureLogEntry.append(m_pressure);
        }
        m_robotContainer.m_drivetrain.outputCalibrationTelemetry();
        SmartDashboard.putNumber("Pressure (PSI)", m_pressure);
        SmartDashboard.putNumber("Compressor (Amps)", m_compressorCurrent);
        CommandScheduler.getInstance().run();
    }

    @Override
    public void disabledInit() {}

    @Override
    public void disabledPeriodic() {}

    @Override
    public void autonomousInit() {}

    @Override
    public void autonomousPeriodic() {}

    @Override
    public void teleopInit() {
        m_compressor.enableDigital();
    }

    @Override
    public void teleopPeriodic() {}

    @Override
    public void testInit() {
        CommandScheduler.getInstance().cancelAll();
    }

    @Override
    public void testPeriodic() {}

    @Override
    public void simulationInit() {}

    @Override
    public void simulationPeriodic() {}
}
