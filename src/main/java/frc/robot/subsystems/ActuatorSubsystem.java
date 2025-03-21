package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.can.WPI_VictorSPX;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
// import frc.robot.Constants.IntakeConstants;
import frc.robot.Constants.ActuatorConstants;

public class ActuatorSubsystem extends SubsystemBase {

    private final WPI_VictorSPX actuatorMotor;

    /**
     * This subsystem controls the actuator.
     */
    public ActuatorSubsystem() {
        // Initialize the actuator motor using its CAN ID
        actuatorMotor = new WPI_VictorSPX(ActuatorConstants.ACTUATOR_MOTOR_ID);
        // actuatorMotor.setInverted(false);
    }

    @Override
    public void periodic() {
        // This method will be called once per scheduler run
    }

    /**
     * Runs the actuator motor at a given speed.
     * Positive values move it forward, negative values move it in reverse.
     * 
     * @param speed Motor speed from -1.0 to 1.0, with 0 stopping it.
     */
    public void runActuator(double speed) {
        actuatorMotor.set(speed);
    }
}
