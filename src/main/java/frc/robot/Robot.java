// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.util.sendable.SendableRegistry;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.PS4Controller;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.motorcontrol.PWMVictorSPX;

/**
 * This is a demo program showing the use of the DifferentialDrive class, specifically it contains
 * the code necessary to operate a robot with tank drive.
 */
public class Robot extends TimedRobot {
  private DifferentialDrive m_robotDrive;
  private PS4Controller m_controller; 

  private final PWMVictorSPX m_leftMotor = new PWMVictorSPX(0);
  private final PWMVictorSPX m_rightMotor = new PWMVictorSPX(1);

  @Override
  public void robotInit() {
    m_robotDrive = new DifferentialDrive(m_leftMotor, m_rightMotor);
    m_controller = new PS4Controller(0);
    SendableRegistry.addChild(m_robotDrive, m_leftMotor);
    DriverStation.reportWarning("Robot initialized", false);
  }

  @Override
  public void robotPeriodic() {
    // Add code to be run every robot packet, no matter the mode.
    // DriverStation.reportWarning("robotPeriodic called", false);
  }

  @Override
  public void disabledPeriodic() {
    // Add code to be run every robot packet when the robot is disabled.
    // DriverStation.reportWarning("disabledPeriodic called", false);
  }

  @Override
  public void teleopPeriodic() {
    m_robotDrive.tankDrive(m_controller.getLeftY(), m_controller.getRightY());

    // DriverStation.reportWarning("left joystick: " + m_leftStick.getY(), false);
    // DriverStation.reportWarning("Right joystick: " + m_rightStick.getY(), false);
  }
}