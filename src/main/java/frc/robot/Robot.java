package frc.robot;

import edu.wpi.first.util.datalog.DataLog;
import edu.wpi.first.util.datalog.DoubleLogEntry;
import edu.wpi.first.util.datalog.StringLogEntry;
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
    private static final boolean m_enableLogger = true;

    private CommandScheduler m_commandScheduler;
    private RobotContainer m_robotContainer;
    private Compressor m_compressor;
    private double m_pressurePsi;
    private double m_imuYawAngleDeg;
    private DoubleLogEntry m_pressureLogEntry;
    private DoubleLogEntry m_imuYawAngleLogEntry;
    private DoubleLogEntry m_compressorCurrentLogEntry;
    private StringLogEntry m_modeLogEntry;

    @Override
    public void robotInit() {
        LiveWindow.disableAllTelemetry();
        m_commandScheduler = CommandScheduler.getInstance();
        m_commandScheduler.disable();
        m_compressor = new Compressor(Constants.Hardware.PCM_ID, PneumaticsModuleType.REVPH);
        m_compressor.disable();
        if (m_enableLogger) {
            DataLogManager.start();
            DataLogManager.logNetworkTables(false);
            DataLog log = DataLogManager.getLog();
            m_pressureLogEntry = new DoubleLogEntry(log, " Pressure (psi)");
            m_compressorCurrentLogEntry = 
                new DoubleLogEntry(log, "Compressor Current (A)");
            m_modeLogEntry = new StringLogEntry(log, "FMS Mode");
            m_imuYawAngleLogEntry = new DoubleLogEntry(log, "IMU Yaw Angle (deg)");
        }
        m_robotContainer = new RobotContainer();
    }

    @Override
    public void autonomousInit() {
        m_commandScheduler.enable();
        if (m_enableLogger) {
            m_modeLogEntry.append("Auto");
        }        
    }

    @Override
    public void autonomousPeriodic() {
        if (m_enableLogger) {
            m_compressorCurrentLogEntry.append(m_compressor.getCurrent());
        }
    }

    @Override
    public void teleopInit() {
        if (m_enableLogger) {
            m_modeLogEntry.append("Teleop");
        }        
        m_compressor.enableDigital();
    }

    @Override
    public void teleopPeriodic() {
        if (m_enableLogger) {
            m_compressorCurrentLogEntry.append(m_compressor.getCurrent());
        }
    }

    @Override
    public void teleopExit() {
        m_commandScheduler.disable();
    }

    @Override
    public void robotPeriodic() {
        m_commandScheduler.run();
        m_pressurePsi = m_compressor.getPressure();
        m_imuYawAngleDeg = m_robotContainer.m_drivetrain.getImuYawAngleDeg();
        if (m_enableLogger) {
            m_pressureLogEntry.append(m_pressurePsi);
            m_imuYawAngleLogEntry.append(m_imuYawAngleDeg);
        }        
        SmartDashboard.putNumber("Pressure (PSI)", m_pressurePsi);
        SmartDashboard.putNumber("IMU Yaw Angle (deg)", m_imuYawAngleDeg);
    }


    @Override
    public void disabledInit() {
        if (m_enableLogger) {
            m_modeLogEntry.append("Disabled");
        }  
    }

    @Override
    public void disabledPeriodic() {}

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
