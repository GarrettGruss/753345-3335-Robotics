package frc.robot;

import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
// import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.Constants.DriveConstants;
import frc.robot.Constants.OperatorConstants;
import frc.robot.autos.DriveForwardAuto;
import frc.robot.autos.SimpleCoralAuto;
// import frc.robot.commands.AlgaeInCommand;
// import frc.robot.commands.AlgaeOutCommand;
import frc.robot.commands.CoralOutCommand;
// import frc.robot.commands.CoralStackCommand;
import frc.robot.commands.DriveCommand;
import frc.robot.commands.ActuatorCommand;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.RollerSubsystem;
import frc.robot.subsystems.ActuatorSubsystem;

public class RobotContainer {

  // Controllers
  private final CommandXboxController m_driverController =
      new CommandXboxController(OperatorConstants.DRIVER_CONTROLLER_PORT);

  // Autonomous chooser
  SendableChooser<Command> m_chooser = new SendableChooser<>();

  // Subsystems
  public final RollerSubsystem m_roller = new RollerSubsystem();
  public final IntakeSubsystem m_intake = new IntakeSubsystem();
  public final DriveSubsystem m_drive = new DriveSubsystem();
  public final ActuatorSubsystem m_actuator = new ActuatorSubsystem(); // Added actuator subsystem

  // Autonomous commands
  public final SimpleCoralAuto m_simpleCoralAuto = new SimpleCoralAuto(m_drive, m_roller);
  public final DriveForwardAuto m_driveForwardAuto = new DriveForwardAuto(m_drive);

  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {
    configureBindings();
    m_chooser.setDefaultOption("Coral Auto", m_simpleCoralAuto);
    m_chooser.addOption("Drive Forward Auto", m_driveForwardAuto);
    SmartDashboard.putData(m_chooser);
  }

  private void configureBindings() {

    // Default drive command
    m_drive.setDefaultCommand(new DriveCommand(m_drive,
        () -> -m_driverController.getLeftY() * 0.8,
        () -> -m_driverController.getLeftX() * 0.9,
        () -> true));

    // Slow mode (hold left bumper)
    m_driverController.leftBumper().whileTrue(new DriveCommand(m_drive, 
        () -> -m_driverController.getLeftY() * DriveConstants.SLOW_MODE_MOVE,  
        () -> -m_driverController.getLeftX() * DriveConstants.SLOW_MODE_TURN,
        () -> true));

    // Roller controls
    m_driverController.rightBumper().whileTrue(new CoralOutCommand(m_roller));
    // m_driverController.y().whileTrue(new CoralStackCommand(m_roller));

    // Actuator control (right Y-axis)
    m_actuator.setDefaultCommand(new ActuatorCommand(m_actuator,
        () -> -m_driverController.getRightY())); // Uses right Y-axis to control actuator
  }

  public Command getAutonomousCommand() {
    return m_chooser.getSelected();
  }
}
