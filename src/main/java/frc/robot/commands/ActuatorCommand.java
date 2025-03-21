package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.ActuatorSubsystem;

import java.util.function.DoubleSupplier;

// Command to move actuator with joystick inputs
public class ActuatorCommand extends Command {

  private final ActuatorSubsystem m_actuator;
  private final DoubleSupplier m_speedSupplier;

  /**
   * Controls the actuator based on joystick input.
   * 
   * @param actuator The actuator subsystem.
   * @param speedSupplier The supplier for joystick input (-1.0 to 1.0).
   */
  public ActuatorCommand(ActuatorSubsystem actuator, DoubleSupplier speedSupplier) {
    m_actuator = actuator;
    m_speedSupplier = speedSupplier;
    addRequirements(m_actuator);
  }

  @Override
  public void execute() {
    m_actuator.runActuator(m_speedSupplier.getAsDouble());
  }

  @Override
  public void end(boolean interrupted) {
    m_actuator.runActuator(0);
  }

  @Override
  public boolean isFinished() {
    return false;
  }
}
