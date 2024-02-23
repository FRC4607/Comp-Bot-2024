package frc.robot.util.ctre;

import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.hardware.core.CoreTalonFX;

import edu.wpi.first.util.datalog.BooleanLogEntry;
import edu.wpi.first.util.datalog.DataLog;
import edu.wpi.first.util.datalog.DoubleLogEntry;
import edu.wpi.first.wpilibj.DataLogManager;
import frc.robot.Robot;

/**
 * A class that manages the logging and storage of useful signals of a single
 * TalonFX object.
 */
public class TalonFXStandardSignalLogger {
    private final CoreTalonFX m_device;

    public final StatusSignal<Double> m_pos;
    private final DoubleLogEntry m_posLog;
    public final StatusSignal<Double> m_torque;
    private final DoubleLogEntry m_torqueLog;
    public final StatusSignal<Double> m_velocity;
    private final DoubleLogEntry m_velocityLog;
    public final StatusSignal<Double> m_acceleration;
    private final DoubleLogEntry m_accelerationLog;

    public final StatusSignal<Double> m_deviceTempSecondary;
    private final DoubleLogEntry m_deviceTempSecondaryLog;
    public final StatusSignal<Double> m_deviceTemp;
    private final DoubleLogEntry m_deviceTempLog;
    public final StatusSignal<Double> m_processorTemp;
    private final DoubleLogEntry m_processorTempLog;

    public final StatusSignal<Boolean> m_deviceTempFault;
    private final BooleanLogEntry m_deviceTempFaultLog;
    public final StatusSignal<Boolean> m_procTempFault;
    private final BooleanLogEntry m_procTempFaultLog;

    /**
     * Creates a new TalonFXStandardSignalLogger.
     * 
     * @param device The {@link com.ctre.phoenix6.hardware.CoreTalonFX} to use.
     * @param prefix A string that will be prefixed to the various log entries.
     *               Should not end with a trailing slash.
     * @param caniv  Whether or not the signals of this object should be refreshed
     *               with the CANivore signals.
     */
    public TalonFXStandardSignalLogger(CoreTalonFX device, String prefix, boolean caniv) {
        m_device = device;

        DataLog managedLog = DataLogManager.getLog();

        m_pos = device.getPosition();
        m_pos.setUpdateFrequency(50.0);
        m_posLog = new DoubleLogEntry(managedLog, prefix + "/pos");
        m_torque = device.getTorqueCurrent();
        m_torque.setUpdateFrequency(50.0);
        m_torqueLog = new DoubleLogEntry(managedLog, prefix + "/torque");
        m_velocity = device.getVelocity();
        m_velocity.setUpdateFrequency(50.0);
        m_velocityLog = new DoubleLogEntry(managedLog, prefix + "/vel");
        m_acceleration = device.getVelocity();
        m_acceleration.setUpdateFrequency(50.0);
        m_accelerationLog = new DoubleLogEntry(managedLog, prefix + "/accel");

        m_deviceTempSecondary = device.getAncillaryDeviceTemp();
        m_deviceTempSecondary.setUpdateFrequency(4.0);
        m_deviceTempSecondaryLog = new DoubleLogEntry(managedLog, prefix + "/tempsecondary");
        m_deviceTemp = device.getDeviceTemp();
        m_deviceTemp.setUpdateFrequency(4.0);
        m_deviceTempLog = new DoubleLogEntry(managedLog, prefix + "/temp");
        m_processorTemp = device.getProcessorTemp();
        m_processorTemp.setUpdateFrequency(4.0);
        m_processorTempLog = new DoubleLogEntry(managedLog, prefix + "/tempproc");

        new BooleanLogEntry(managedLog, prefix + "/faults/bootduringenable", prefix)
                .append(device.getStickyFault_BootDuringEnable().getValue());
        new BooleanLogEntry(managedLog, prefix + "/faults/hardware", prefix)
                .append(device.getStickyFault_Hardware().getValue());

        m_deviceTempFault = device.getFault_DeviceTemp();
        m_deviceTempFault.setUpdateFrequency(4.0);
        m_deviceTempFaultLog = new BooleanLogEntry(managedLog, prefix + "/faults/devicetemp");
        m_procTempFault = device.getFault_ProcTemp();
        m_procTempFault.setUpdateFrequency(4.0);
        m_procTempFaultLog = new BooleanLogEntry(managedLog, prefix + "/faults/proctemp");

        if (caniv) {
            Robot.addSignalsCaniv(
                    m_pos,
                    m_torque,
                    m_velocity,
                    m_acceleration,
                    m_deviceTempSecondary,
                    m_deviceTemp,
                    m_processorTemp,
                    m_deviceTempFault,
                    m_procTempFault);
        } else {
            Robot.addSignalsRio(
                    m_pos,
                    m_torque,
                    m_velocity,
                    m_acceleration,
                    m_deviceTempSecondary,
                    m_deviceTemp,
                    m_processorTemp,
                    m_deviceTempFault,
                    m_procTempFault);
        }

        m_device.clearStickyFaults();
    }

    /**
     * Creates a new TalonFXStandardSignalLogger. The device provided must be on the
     * RIO CAN bus for correct functionality.
     * 
     * @param device The {@link com.ctre.phoenix6.hardware.CoreTalonFX} to use.
     * @param prefix A string that will be prefixed to the various log entries.
     *               Should not end with a trailing slash.
     */
    public TalonFXStandardSignalLogger(CoreTalonFX device, String prefix) {
        this(device, prefix, false);
    }

    /**
     * Writes the current values of the signals to the log with their timestamps.
     * This method should be called in the {@code periodic} method of a
     * {@link edu.wpi.first.wpilibj2.command.Subsystem}.
     */
    public void log() {
        m_posLog.append(m_pos.getValueAsDouble());
        m_torqueLog.append(m_torque.getValueAsDouble());
        m_velocityLog.append(m_velocity.getValueAsDouble());
        m_accelerationLog.append(m_acceleration.getValueAsDouble());
        m_deviceTempLog.append(m_deviceTemp.getValueAsDouble());
        m_deviceTempSecondaryLog.append(m_deviceTempSecondary.getValueAsDouble());
        m_processorTempLog.append(m_processorTemp.getValueAsDouble());

        m_deviceTempFaultLog.append(m_deviceTempFault.getValue());
        m_procTempFaultLog.append(m_procTempFault.getValue());
    }
}
