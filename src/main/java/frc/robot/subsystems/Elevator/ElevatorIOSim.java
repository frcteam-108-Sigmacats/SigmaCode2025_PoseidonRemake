package frc.robot.subsystems.Elevator;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N2;
import edu.wpi.first.math.system.LinearSystem;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.simulation.ElevatorSim;

public class ElevatorIOSim implements ElevatorIO {
  // Creates the Elevator System Simulation
  private LinearSystem<N2, N1, N2> elevatorSystem;
  // Creates a basic Elevator Simulation System
  private ElevatorSim elevatorSim;

  // Creates PID Controller to set elevator to positions
  private PIDController elevatorPID;

  // Creates a variable to stor the Elevator Voltage Output
  private double elevatorMotorVoltage = 0.0;

  public ElevatorIOSim() {
    // Creates Elevator System using information from actual robot to simulate
    elevatorSystem =
        LinearSystemId.createElevatorSystem(
            DCMotor.getNEO(2), Units.lbsToKilograms(25.6), Units.inchesToMeters(0.54), 12);

    // Creates Elevator System using information from actual robot
    elevatorSim =
        new ElevatorSim(
            elevatorSystem,
            DCMotor.getNEO(2).withReduction(12),
            0,
            Units.inchesToMeters(30),
            true,
            Units.inchesToMeters(0),
            0.0,
            0.0);

    // Assigns Elevator PID Controller to set Elevator to positions
    elevatorPID = new PIDController(30.0, 0.0, 0);
  }

  // Updates inputs assigining them to simulated components
  @Override
  public void updateInputs(ElevatorIOInputs inputs) {
    // Assigns voltage inputs to variable storing the voltage readings
    inputs.leftElevatorMotorVoltage = elevatorMotorVoltage;
    inputs.rightElevatorMotorVoltage = elevatorMotorVoltage;
    // Assigns the current draw of the motors from Elevator Sim
    inputs.leftElevatorMotorCurrentAmps = elevatorSim.getCurrentDrawAmps();
    inputs.rightElevatorMotorCurrentAmps = elevatorSim.getCurrentDrawAmps();
    // Assigns the elevator position from the Elevator Sim
    inputs.elevatorPos = Units.metersToFeet(elevatorSim.getPositionMeters());

    // Sets the Inputs Voltage of the Elevator Simulation to simulate the elevator movement
    elevatorSim.setInputVoltage(elevatorMotorVoltage);
    // Updates Elevator Simulated every 0.02s
    elevatorSim.update(0.02);
  }

  // Sets the Elevator Position
  @Override
  public void setElevatorPosition(double position) {
    elevatorMotorVoltage =
        elevatorPID.calculate(Units.metersToFeet(elevatorSim.getPositionMeters()), position / 2);
  }

  @Override
  public void setElevatorSpeed(double speed) {
    elevatorMotorVoltage = 12 * speed;
  }
}
