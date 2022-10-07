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
 * @see TimedRobot
 */
public class Robot extends TimedRobot {

    private Compressor m_compressor;
    private double m_pressure;
    private DoubleLogEntry m_pressureLogEntry;

    //private RobotContainer m_robotContainer;


    @Override
    public void robotInit() {
        //m_robotContainer = new RobotContainer();
        m_compressor = new Compressor(Constants.Hardware.PCM_ID, PneumaticsModuleType.REVPH);
        m_compressor.disable();
        LiveWindow.disableAllTelemetry();


        // Use the new data logger to log the IMU heading and temperature. Be sure to always use a
        // USB thumb drive for logging. To do this, be sure to format the thumb drive with a
        // RoboRIO compatible FS (like FAT32) and simply plug into one of the 2 USB ports on the
        // RoboRIO. See https://docs.wpilib.org/en/stable/docs/software/telemetry/datalog.html
        DataLogManager.start();
        DataLogManager.logNetworkTables(false);
        DataLog log = DataLogManager.getLog();
        m_pressureLogEntry = new DoubleLogEntry(log, "/sensors/pressure");

    }

    @Override
    public void robotPeriodic() {
        m_pressure = m_compressor.getPressure(); // pressure = 250 * (Vout/Vcc) - 25
        //m_compressor.getAnalogVoltage();
        m_pressureLogEntry.append(m_pressure);
        SmartDashboard.putNumber("Pressure (???)", m_pressure);
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
