// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.SignalLogger;

import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;

public class Robot extends TimedRobot {
    private static BaseStatusSignal[] m_signalsToRefreshRio = {};
    private static BaseStatusSignal[] m_signalsToRefreshCaniv = {};
    private Command m_autonomousCommand;

    private RobotContainer m_robotContainer;

    /**
     * Adds signals to be refreshed before every loop. The signals provided should
     * be on the RIO's CAN bus.
     * 
     * @param signals The signals to be refreshed.
     */
    public static void addSignalsRio(BaseStatusSignal... signals) {
        BaseStatusSignal[] newSignals = new BaseStatusSignal[m_signalsToRefreshRio.length + signals.length];
        System.arraycopy(m_signalsToRefreshRio, 0, newSignals, 0, m_signalsToRefreshRio.length);
        System.arraycopy(signals, 0, newSignals, m_signalsToRefreshRio.length, signals.length);
        m_signalsToRefreshRio = newSignals;
    }

    /**
     * Adds signals to be refreshed before every loop. The signals provided should
     * be on the CANivore's CAN bus.
     * 
     * @param signals The signals to be refreshed.
     */
    public static void addSignalsCaniv(BaseStatusSignal... signals) {
        BaseStatusSignal[] newSignals = new BaseStatusSignal[m_signalsToRefreshCaniv.length + signals.length];
        System.arraycopy(m_signalsToRefreshCaniv, 0, newSignals, 0, m_signalsToRefreshCaniv.length);
        System.arraycopy(signals, 0, newSignals, m_signalsToRefreshCaniv.length, signals.length);
        m_signalsToRefreshCaniv = newSignals;
    }

    @Override
    public void robotInit() {
        SignalLogger.stop();
        m_robotContainer = new RobotContainer();
    }

    @Override
    public void robotPeriodic() {
        // Refresh every signal before running loop
        BaseStatusSignal.refreshAll(m_signalsToRefreshRio);
        BaseStatusSignal.refreshAll(m_signalsToRefreshCaniv);
        CommandScheduler.getInstance().run();
    }

    @Override
    public void disabledInit() {
    }

    @Override
    public void disabledPeriodic() {
    }

    @Override
    public void disabledExit() {
    }

    @Override
    public void autonomousInit() {
        m_autonomousCommand = m_robotContainer.getAutonomousCommand();

        if (m_autonomousCommand != null) {
            m_autonomousCommand.schedule();
        }
    }

    @Override
    public void autonomousPeriodic() {
    }

    @Override
    public void autonomousExit() {
    }

    @Override
    public void teleopInit() {
        if (m_autonomousCommand != null) {
            m_autonomousCommand.cancel();
        }
    }

    @Override
    public void teleopPeriodic() {
    }

    @Override
    public void teleopExit() {
    }

    @Override
    public void testInit() {
        CommandScheduler.getInstance().cancelAll();
    }

    @Override
    public void testPeriodic() {
    }

    @Override
    public void testExit() {
    }

    @Override
    public void simulationPeriodic() {
    }
}
