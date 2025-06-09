package frc.robot.subsystems.Wrist;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N2;
import edu.wpi.first.math.system.LinearSystem;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.units.Units;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.wpilibj.simulation.DCMotorSim;
import edu.wpi.first.wpilibj.simulation.DIOSim;
import org.ironmaple.simulation.IntakeSimulation;
import org.ironmaple.simulation.SimulatedArena;
import org.ironmaple.simulation.drivesims.AbstractDriveTrainSimulation;

public class CXAMechIOSim implements CXAMechIO {
  // Creates the Sim Motors that will be on the Wrist Mech
  private DCMotorSim cxaMotor;
  private DCMotorSim hopperMotor;
  private DCMotorSim wristMotor;

  // Creates the PID Controllers to manage the controllers set speed like the built in ones
  private PIDController wristPivot;
  private PIDController cxaVelocityControl;

  // Creates a IR Sensor Sim to use
  private DIOSim canRange;

  // Creating the physical linear system that will mock the real motor's environment
  private LinearSystem<N2, N1, N2> cxaMotorSystem =
      LinearSystemId.createDCMotorSystem(DCMotor.getKrakenX60(1), 1.02, 4);
  private LinearSystem<N2, N1, N2> hopperMotorSystem =
      LinearSystemId.createDCMotorSystem(DCMotor.getNEO(1), 0.5, 12);
  private LinearSystem<N2, N1, N2> wristMotorSystem =
      LinearSystemId.createSingleJointedArmSystem(DCMotor.getNeoVortex(1), 2.5, 56.8);

  // Constructs some voltage readers to send to the inputs being logged
  private double cxaMotorVoltage;
  private double hopperMotorVoltage;
  private double wristMotorVoltage;

  // Creates a physic generated intake simulation for intaking coral from ground
  private IntakeSimulation coralIntakeSimulation;

  private IntakeSimulation algaeIntakeSimulation;

  public CXAMechIOSim(AbstractDriveTrainSimulation driveTrainSimulation) {
    // Constructs the Motor Simulations given basic information about the given motors and their
    // mounting situation
    cxaMotor = new DCMotorSim(cxaMotorSystem, DCMotor.getKrakenX60(1).withReduction(4), 0.0, 0.0);
    hopperMotor = new DCMotorSim(hopperMotorSystem, DCMotor.getNEO(1).withReduction(12), 0.0, 0.0);
    wristMotor =
        new DCMotorSim(wristMotorSystem, DCMotor.getNeoVortex(1).withReduction(56.8), 0.0, 5.0);

    // Creates a PID that will simulate Position and Velocity Control on the robot
    wristPivot = new PIDController(1.0, 0, 0.0);
    wristPivot.enableContinuousInput(0, 360);
    cxaVelocityControl = new PIDController(0.01, 0, 0);

    // Assigned the IR Sensor to Simulated Channel
    canRange = new DIOSim(0);

    // Sets the IR Sensor to default value of false
    canRange.setValue(false);

    // Creates the physic intake simulation given certain measurements of hopper and where its
    // position
    this.coralIntakeSimulation =
        IntakeSimulation.InTheFrameIntake(
            "Coral",
            driveTrainSimulation,
            Distance.ofRelativeUnits(23.5, Units.Inches),
            IntakeSimulation.IntakeSide.BACK,
            1);

    this.algaeIntakeSimulation =
        IntakeSimulation.InTheFrameIntake(
            "Algae",
            driveTrainSimulation,
            Distance.ofRelativeUnits(6, Units.Inches),
            IntakeSimulation.IntakeSide.FRONT,
            1);
  }

  // Updates the inputs of the simulated motors
  public void updateInputs(CXAMechIOInputs inputs) {
    // Assings the motor voltages to the variables created in the class
    inputs.cxaMotorVoltage = cxaMotorVoltage;
    inputs.cxaMotorCurrentAmps = inputs.detectAlgae ? 35 : 0;
    inputs.hopperMotorVoltage = hopperMotorVoltage;
    inputs.cxaWristVoltage = wristMotorVoltage;

    // Assings the Wrist Angle to the simulated motor current position
    inputs.wristAngle = Math.IEEEremainder(wristMotor.getAngularPosition().in(Units.Degree), 360);

    // Assigns the boolean to the simulated IR Sensor value
    inputs.detectCoral = canRange.getValue();

    // Assigns the boolean to Detect Algae based on Intake Simulation
    inputs.detectAlgae = algaeIntakeSimulation.getGamePiecesAmount() == 1;

    cxaMotor.setInputVoltage(cxaMotorVoltage);

    cxaMotor.update(SimulatedArena.getSimulationDt().in(Units.Seconds));

    // Sets the motor input voltage to tell it that it to move
    wristMotor.setInputVoltage(wristMotorVoltage);
    // Updates the motor every 0.02s
    wristMotor.update(SimulatedArena.getSimulationDt().in(Units.Seconds));

    // Sets the CANRange to the intake simulation game piece tracker
    canRange.setValue(coralIntakeSimulation.getGamePiecesAmount() == 1 ? true : false);
  }

  // Sets the motor velocity for the CXA Motor
  @Override
  public void setCXAMotorVel(double velocity, int slot) {
    cxaMotorVoltage = cxaVelocityControl.calculate(cxaMotor.getAngularVelocityRPM(), velocity);
    if (cxaMotorVoltage <= 0 && slot == 0) {
      coralIntakeSimulation.setGamePiecesCount(0);
      canRange.setValue(false);
    }
    if (cxaMotorVoltage <= 0 && slot == 1) {
      algaeIntakeSimulation.startIntake();
    }
    if (cxaMotorVoltage >= 0) {
      algaeIntakeSimulation.setGamePiecesCount(0);
    }
  }

  // Sets the Position of the Wrist Angle using PID Controller and wrist motor readings
  @Override
  public void setWristPosition(double position) {
    wristMotorVoltage =
        wristPivot.calculate(
            Math.IEEEremainder(wristMotor.getAngularPosition().in(Units.Degree), 360), position);
    wristMotor.setAngularVelocity(wristMotorVoltage * (2 * Math.PI / 60));
  }

  // Sets the hopper speed where it also tells the physic generated intake to run to intake the
  // coral
  @Override
  public void setHopperSpeed(double speed) {
    if (speed != 0) {
      coralIntakeSimulation.startIntake();
    } else {
      coralIntakeSimulation.stopIntake();
    }
  }

  // Sets the wrist speed manually
  @Override
  public void setWristSpeed(double speed) {
    wristMotorVoltage = 12 * speed;
  }

  // Sets the CXA Motor Speed manually
  @Override
  public void setCXAMotorSpeed(double speed) {
    cxaMotorVoltage = 12 * speed;
    if (speed == 0) {
      algaeIntakeSimulation.stopIntake();
    }
  }
}
