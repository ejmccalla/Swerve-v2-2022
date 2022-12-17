package frc.robot;

import edu.wpi.first.hal.can.CANStatus;
import edu.wpi.first.util.datalog.BooleanLogEntry;
import edu.wpi.first.util.datalog.DataLog;
import edu.wpi.first.util.datalog.DoubleLogEntry;
import edu.wpi.first.util.datalog.IntegerLogEntry;
import edu.wpi.first.util.datalog.StringLogEntry;
import edu.wpi.first.wpilibj.Compressor;
import edu.wpi.first.wpilibj.DataLogManager;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.PowerDistribution;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.livewindow.LiveWindow;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
// import frc.robot.commands.CalibrateTurnFF;
// import frc.robot.commands.CalibrateWheelDiameter;
import frc.robot.Constants.Hardware;
import frc.robot.commands.HomeSwerveModules;


/**
 * This is the top-level class where the {@link edu.wpi.first.wpilibj.TimedRobot} states are defined.
 *
 * <p>This project is focused on testing the new mechanical design features used during the robot design and build as well
 * as getting experience with the swerve software. Most notably, the swerve drive control will be leveraging the provided
 * WPILib swerve functionality and exteded with thePath Planner software for path following. Another big part to the
 * software is the extensive use of the WPILib data logger. This is used to log telemetry and other useful data for analysis
 * (failure prediction and understaning, system tuning, and etc). Be sure to always use a USB thumb drive while logging. To
 * do this, be sure to format the thumb drive with a RoboRIO compatible FS (like FAT32) and simply plug into one of the two
 * USB ports on the RoboRIO.
 *
 * <p>The states are run based on {@link edu.wpi.first.wpilibj.IterativeRobotBase#loopFunc()}
 *
 * <p>This project uses the WPILib {@link edu.wpi.first.wpilibj2.command.CommandScheduler}. The
 * {@link edu.wpi.first.wpilibj2.command.CommandScheduler#run()} method of the scheduler is called every iteration of the
 * perdiodic loop.
 *
 * <p>The REV Robotics pressure sensor can be used for closed-loop control per the 2022 FRC game rules.
 *
 * @see <a href="https://github.com/mjansen4857/pathplanner">Path Planner</a>
 * @see <a href="https://docs.wpilib.org/en/stable/docs/software/telemetry/datalog.html">WPILib Logger</a>
 */
public class Robot extends TimedRobot {

    private static final double LOOP_TIME_TO_HOURS = 0.02 / 3600.0;
    private static final boolean m_enableLogger = true;
    private CommandScheduler m_commandScheduler;
    private PowerDistribution m_pdh;
    private RobotContainer m_robotContainer;
    private Compressor m_compressor;
    private double m_pressurePsi;
    private double m_imuYawAngleDeg;
    private double m_imuTempDegC;
    private double m_totalCurrentA;
    private double m_voltageV;
    private double m_totalPowerWHs;
    private CANStatus m_canStatus;
    private DoubleLogEntry m_pressureLogEntry;
    private DoubleLogEntry m_imuYawAngleLogEntry;
    private DoubleLogEntry m_imuTempLogEntry;
    private DoubleLogEntry m_compressorCurrentLogEntry;
    private StringLogEntry m_modeLogEntry;
    private BooleanLogEntry m_roborioBrownedOutLogEntry;
    private DoubleLogEntry m_roborioCanUtilizationLogEntry;
    private IntegerLogEntry m_roborioCanOffCountLogEntry;
    private IntegerLogEntry m_roborioCanRxErrCountLogEntry;
    private IntegerLogEntry m_roborioCanTxErrCountLogEntry;
    private IntegerLogEntry m_roborioCanTxFullCountLogEntry;
    private BooleanLogEntry m_roborioStaleDsDataLogEntry;
    private DoubleLogEntry m_pdhTotalCurrentDoubleLogEntry;
    private DoubleLogEntry m_pdhInputVoltageDoubleLogEntry;
    private DoubleLogEntry m_pdhTotalPowerDoubleLogEntry;

    /**
     * This method is called only a single time when the robot is first powered on. This is where initialization code
     * should go.
     *
     * <p>1. Disable all of the LiveWindow telemetry since it's not used and having it only eats up bandwidth
     *
     * <p>2. Disable the compressor to keep it from turning on during autonomous. The path-following is tuned with the
     * compressor off, so keep it off during auto to maintain accuracy.
     * 
     * <p>3. If the logger is being used, start the log manager and setup all of the log entries. Each of the subsystems
     * will have their individual control and setup.
     * 
     * <p>4. Instantiate the robot container and thus all of the subsystems.
     * 
     * <p>5. Disable the command scheduler to keep the subsystems from logging data (run the scheculer once to initialize
     * the logging and avoid loop overruns). Any data which needs to be logged while the robot is disabled should be done so
     * in this class.
     */
    @Override
    public void robotInit() {
        LiveWindow.disableAllTelemetry();
        m_pdh = new PowerDistribution(Hardware.REV_PDH_ID, PowerDistribution.ModuleType.kRev);
        m_robotContainer = new RobotContainer();
        m_compressor = new Compressor(Constants.Hardware.REV_PH_ID, PneumaticsModuleType.REVPH);
        m_compressor.disable();
        m_pressurePsi = m_compressor.getPressure();
        m_imuYawAngleDeg = m_robotContainer.m_drivetrain.getImuYawAngleDeg();
        m_imuTempDegC = m_robotContainer.m_drivetrain.getImuTempDegC();
        m_totalCurrentA = m_pdh.getTotalCurrent();
        m_voltageV = m_pdh.getVoltage();
        m_totalPowerWHs = m_totalCurrentA * m_voltageV * 0.02;
        m_canStatus = RobotController.getCANStatus();
        SmartDashboard.putNumber("Pressure (PSI)", m_pressurePsi);
        SmartDashboard.putNumber("IMU Yaw Angle (deg)", m_imuYawAngleDeg);        
        if (m_enableLogger) {
            DataLogManager.start();
            DataLogManager.logNetworkTables(false);
            DataLog log = DataLogManager.getLog();
            m_pressureLogEntry = new DoubleLogEntry(log, "Pressure (psi)");
            m_pressureLogEntry.append(m_pressurePsi);
            m_compressorCurrentLogEntry = new DoubleLogEntry(log, "Compressor Current (A)");
            m_compressorCurrentLogEntry.append(m_compressor.getCurrent());
            m_imuYawAngleLogEntry = new DoubleLogEntry(log, "IMU Yaw Angle (deg)");
            m_imuYawAngleLogEntry.append(m_imuYawAngleDeg);
            m_imuTempLogEntry = new DoubleLogEntry(log, "IMU Temp (degC");
            m_imuTempLogEntry.append(m_imuTempDegC);
            m_roborioBrownedOutLogEntry = new BooleanLogEntry(log, "RoboRio Browned Out");
            m_roborioBrownedOutLogEntry.append(RobotController.isBrownedOut());
            m_roborioCanUtilizationLogEntry = new DoubleLogEntry(log, "RoboRio CAN Utilization");
            m_roborioCanUtilizationLogEntry.append(m_canStatus.percentBusUtilization);
            m_roborioCanOffCountLogEntry = new IntegerLogEntry(log, "RoboRio CAN Off Count");
            m_roborioCanOffCountLogEntry.append(m_canStatus.busOffCount);
            m_roborioCanRxErrCountLogEntry = new IntegerLogEntry(log, "RoboRio CAN Rx Error Count");
            m_roborioCanRxErrCountLogEntry.append(m_canStatus.receiveErrorCount);
            m_roborioCanTxErrCountLogEntry = new IntegerLogEntry(log, "RoboRio CAN Tx Error Count");
            m_roborioCanTxErrCountLogEntry.append(m_canStatus.transmitErrorCount);
            m_roborioCanTxFullCountLogEntry = new IntegerLogEntry(log, "RoboRio CAN Tx Full Count");
            m_roborioCanTxFullCountLogEntry.append(m_canStatus.txFullCount);
            m_roborioStaleDsDataLogEntry = new BooleanLogEntry(log, "RoboRio Stale DS Data Count");
            m_roborioStaleDsDataLogEntry.append(DriverStation.isNewControlData());
            m_modeLogEntry = new StringLogEntry(log, "FMS Mode");
            m_pdhTotalCurrentDoubleLogEntry = new DoubleLogEntry(log, "PDH Total Current (A)");
            m_pdhTotalCurrentDoubleLogEntry.append(m_totalCurrentA);
            m_pdhInputVoltageDoubleLogEntry = new DoubleLogEntry(log, "PDH Input Voltage (V)");
            m_pdhInputVoltageDoubleLogEntry.append(m_voltageV);
            m_pdhTotalPowerDoubleLogEntry = new DoubleLogEntry(log, "PDH Total Power (W)");
            m_pdhTotalPowerDoubleLogEntry.append(m_totalPowerWHs);
        }
        m_commandScheduler = CommandScheduler.getInstance();
        m_commandScheduler.setPeriod(10);
        m_commandScheduler.run();
        m_commandScheduler.disable();
        m_commandScheduler.setPeriod(0.020);
    }

    /**
     * This method is called only a single time at the start of autonomous. This is where the autonomous-specific
     * initialization code should go.
     *
     * <p>1. Enable the command scheduler. This will begin logging all of the subsytem telemetry, running the subsystem
     * state machines, and processing commands.
     * 
     * <p>2. The swerve module allignment needs to be handled. There are two choices: seed the relative azimuth encoder with
     * the absolute azimuth encoder or home the wheels with the absolute azimuth encoder and set the relative azimuth
     * encoder to 0 once homed. The first is ideal (not spending precious auto time homing the wheels), but it requires care
     * to ensure the absolute azimuth encoder reading has reached steady-state before seeding the relative encoder. For now,
     * run the homing sequence. This is accomplished by intializing the drivetrain state to <b>Homing</b>.
     *
     * <p>3. TODO: autonomous command chooser
     */
    @Override
    public void autonomousInit() {
        m_commandScheduler.enable();
        // m_commandScheduler.schedule(false, new HomeSwerveModules(m_robotContainer.m_drivetrain));
        if (m_enableLogger) {
            m_modeLogEntry.append("Auto");
        }
    }

    /**
     * This method is called every loop during autonomous. It is called prior to the robotPeriodic method and should contain
     * autonomous-specific periodic code.
     *
     * <p>1. Log the compressor current. This should be disabled, so the expectation are the measurements will all be 0 amps.
     */
    @Override
    public void autonomousPeriodic() {
        if (m_enableLogger) {
            m_compressorCurrentLogEntry.append(m_compressor.getCurrent());
        }
    }


    /**
     * This method is called only a single time at the start of teleop. This is where the teleop-specific initialization
     * code should go.
     *
     * <p>1. Enable the command scheduler. During match-play, it will already be enabled, but during driver practice and
     * bring-up the auto cycle is rarely run.
     * 
     * <p>2. Enable the compressor.
     */
    @Override
    public void teleopInit() {
        m_commandScheduler.enable();
        m_commandScheduler.schedule(false, new HomeSwerveModules(m_robotContainer.m_drivetrain));
        if (m_enableLogger) {
            m_modeLogEntry.append("Teleop");
        }        
        m_compressor.enableDigital();
    }


    /**
     * This method is called every loop during teleop. It is called prior to the robotPeriodic method and should contain
     * teleop-specific periodic code.
     *
     * <p>1. Log the compressor current. TODO: optimize power savings by conrolling the compressor based on the current
     * pressure sensor reading and an air usage model.
     */
    @Override
    public void teleopPeriodic() {
        if (m_enableLogger) {
            m_compressorCurrentLogEntry.append(m_compressor.getCurrent());
        }
    }


    /**
     * This method is called only a single time at the end of teleop. This is where the teleop-specific clean-up code should
     * go.
     *
     * <p>1. Disable the command scheduler to keep the subsystems from logging data.
     *
     * <p>2. TODO: revisit the logging here, may need more frequent sampling for some of the metrics.
     */
    @Override
    public void teleopExit() {
        m_commandScheduler.disable();
        m_canStatus = RobotController.getCANStatus();
        m_roborioCanUtilizationLogEntry.append(m_canStatus.percentBusUtilization);
        m_roborioCanOffCountLogEntry.append(m_canStatus.busOffCount);
        m_roborioCanRxErrCountLogEntry.append(m_canStatus.receiveErrorCount);
        m_roborioCanTxErrCountLogEntry.append(m_canStatus.transmitErrorCount);
        m_roborioCanTxFullCountLogEntry.append(m_canStatus.txFullCount);
    }


    /**
     * This method is called every loop during autonomous, teleop, and disabled. Periodic code which is common to all of
     * these robot states should go here.
     *
     * <p>1. Run the command scheduler. This will call the periodic methods of the subsystems, process the new driver and
     * operator requests, and process current commands.
     * 
     * <p>2. Log the pressure and IMU yaw angle. These should be monitored throughout the whole life-cycle of the robot.
     * 
     * <p>3. Send the pressure and IMU yaw angle to the smart dashboard. These are two of the very few data which is useful
     * to the drive team during match-play. 
     */
    @Override
    public void robotPeriodic() {
        m_commandScheduler.run();
        m_pressurePsi = m_compressor.getPressure();
        m_imuYawAngleDeg = m_robotContainer.m_drivetrain.getImuYawAngleDeg();
        m_imuTempDegC = m_robotContainer.m_drivetrain.getImuTempDegC();
        m_totalCurrentA = m_pdh.getTotalCurrent();
        m_voltageV = m_pdh.getVoltage();
        m_totalPowerWHs += m_totalCurrentA * m_voltageV * LOOP_TIME_TO_HOURS;
        if (m_enableLogger) {
            m_pressureLogEntry.append(m_pressurePsi);
            m_imuYawAngleLogEntry.append(m_imuYawAngleDeg);
            m_imuTempLogEntry.append(m_imuTempDegC);
            m_roborioBrownedOutLogEntry.append(RobotController.isBrownedOut());
            m_roborioStaleDsDataLogEntry.append(DriverStation.isNewControlData());
            m_pdhTotalCurrentDoubleLogEntry.append(m_totalCurrentA);
            m_pdhInputVoltageDoubleLogEntry.append(m_voltageV);
            m_pdhTotalPowerDoubleLogEntry.append(m_totalPowerWHs);
        }
        SmartDashboard.putNumber("Pressure (PSI)", m_pressurePsi);
        SmartDashboard.putNumber("IMU Yaw Angle (deg)", m_imuYawAngleDeg);
        SmartDashboard.putNumber("Total Current (A)", m_totalCurrentA);
    }


    /**
     * This method is called only a single time at the start of being disabled. This is where any common cleanup code should
     * go.
     *
     * <p>1. Log the state.
     */
    @Override
    public void disabledInit() {
        if (m_enableLogger) {
            m_modeLogEntry.append("Disabled");
        }
    }

    /**
     * This method is called every loop while the robot is disabled.
     */
    @Override
    public void disabledPeriodic() {}

    /**
     * This method is being used to run calibration commands.
     */
    @Override
    public void testInit() {
        LiveWindow.disableAllTelemetry();
        m_commandScheduler.cancelAll();
        // m_commandScheduler.enable();
        // m_commandScheduler.schedule(false,
        //     new CalibrateWheelDiameter(m_robotContainer.m_drivetrain));
        // m_commandScheduler.schedule(new CalibrateTurnFF(m_robotContainer.m_drivetrain));
    }

    @Override
    public void testPeriodic() {
        // m_robotContainer.m_drivetrain.setModulesTurnVoltage(4.0);
    }

    @Override
    public void simulationInit() {}

    @Override
    public void simulationPeriodic() {}
}
