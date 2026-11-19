// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package first.robot.mechanisms.elevator;

import static org.wpilib.units.Units.Amps;
import static org.wpilib.units.Units.Inches;
import static org.wpilib.units.Units.Rotations;
import static org.wpilib.units.Units.Volts;

import org.wpilib.epilogue.Epilogue;
import org.wpilib.math.system.DCMotor;
import org.wpilib.simulation.ElevatorSim;
import org.wpilib.telemetry.Telemetry;
import org.wpilib.units.measure.Angle;
import org.wpilib.units.measure.AngularAcceleration;
import org.wpilib.units.measure.AngularVelocity;
import org.wpilib.units.measure.Current;
import org.wpilib.units.measure.Distance;
import org.wpilib.units.measure.Temperature;
import org.wpilib.units.measure.Voltage;
import org.wpilib.math.util.Units;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.MotorAlignmentValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.ctre.phoenix6.sim.ChassisReference;
import com.ctre.phoenix6.sim.TalonFXSimState;
import com.ctre.phoenix6.sim.TalonFXSimState.MotorType;

import first.robot.Robot;

/** Add your docs here. */
public class ElevatorIOTalonFX implements ElevatorIO {
    private final TalonFX motor_master = new TalonFX(ElevatorMap.elevator_master, ElevatorMap.elevator_bus);
    private final TalonFX motor_follower = new TalonFX(ElevatorMap.elevator_follower, ElevatorMap.elevator_bus);
    private final TalonFXConfiguration configuration = new TalonFXConfiguration();
    private TalonFXSimState sim_master;
    private TalonFXSimState sim_follower; 

    private ElevatorSim sim_mechanism;

    private final PositionVoltage control_position = new PositionVoltage(0.0);
    private final VoltageOut control_voltage = new VoltageOut(0.0);

    private final StatusSignal<Angle> signal_master_position;
    private final StatusSignal<AngularVelocity> signal_master_velocity;
    private final StatusSignal<AngularAcceleration> signal_master_acceleration;
    private final StatusSignal<Voltage> signal_master_voltage_output;
    private final StatusSignal<Current> signal_master_stator;
    private final StatusSignal<Current> signal_master_supply;
    private final StatusSignal<Temperature> signal_master_temperature;

    private final StatusSignal<Angle> signal_follower_position;
    private final StatusSignal<AngularVelocity> signal_follower_velocity;
    private final StatusSignal<AngularAcceleration> signal_follower_acceleration;
    private final StatusSignal<Voltage> signal_follower_voltage_output;
    private final StatusSignal<Current> signal_follower_stator;
    private final StatusSignal<Current> signal_follower_supply;
    private final StatusSignal<Temperature> signal_follower_temperature;

    private Voltage inputVoltage = Volts.zero();

    public ElevatorIOTalonFX() {

        configuration.CurrentLimits.StatorCurrentLimit = ElevatorConstants.motor_stator_limit.in(Amps);
        configuration.CurrentLimits.StatorCurrentLimitEnable = true;
        configuration.CurrentLimits.SupplyCurrentLimit = ElevatorConstants.motor_supply_limit.in(Amps);
        configuration.CurrentLimits.SupplyCurrentLimitEnable = true;

        configuration.MotorOutput.NeutralMode = NeutralModeValue.Brake;
        configuration.MotorOutput.Inverted = ElevatorConstants.controls_positive_direction;

        configuration.Feedback.SensorToMechanismRatio = ElevatorConstants.drum_conversion_factor;

        configuration.Slot0.kP = ElevatorConstants.controls_pid_kp;
        configuration.Slot0.kI = ElevatorConstants.controls_pid_ki;
        configuration.Slot0.kD = ElevatorConstants.controls_pid_kd;
        configuration.Slot0.kS = ElevatorConstants.controls_pid_ks;
        configuration.Slot0.kV = ElevatorConstants.controls_pid_kv;
        configuration.Slot0.kG = ElevatorConstants.controls_pid_kg;

        configuration.Voltage.PeakForwardVoltage = 12.0;
        configuration.Voltage.PeakReverseVoltage = -12.0;

        configuration.MotorOutput.PeakForwardDutyCycle = 1.0;
        configuration.MotorOutput.PeakReverseDutyCycle = -1.0;

        motor_master.getConfigurator().apply(configuration, 0.25);
        motor_follower.getConfigurator().apply(configuration, 0.25);

        signal_master_position = motor_master.getPosition();
        signal_master_velocity = motor_master.getVelocity();
        signal_master_acceleration = motor_master.getAcceleration();
        signal_master_voltage_output = motor_master.getMotorVoltage();
        signal_master_stator = motor_master.getStatorCurrent();
        signal_master_supply = motor_master.getSupplyCurrent();
        signal_master_temperature = motor_master.getDeviceTemp();

        signal_follower_position = motor_follower.getPosition();
        signal_follower_velocity = motor_follower.getVelocity();
        signal_follower_acceleration = motor_follower.getAcceleration();
        signal_follower_voltage_output = motor_follower.getMotorVoltage();
        signal_follower_stator = motor_follower.getStatorCurrent();
        signal_follower_supply = motor_follower.getSupplyCurrent();
        signal_follower_temperature = motor_follower.getDeviceTemp();

        // Replace this with how fast the robot controller actually runs
        BaseStatusSignal.setUpdateFrequencyForAll(
            1000.0, 
            signal_master_position,
            signal_master_velocity,
            signal_master_acceleration,
            signal_master_voltage_output,
            signal_master_stator,
            signal_master_supply,
            signal_master_temperature, 
            signal_follower_position,
            signal_follower_velocity,
            signal_follower_acceleration,
            signal_follower_voltage_output,
            signal_follower_stator,
            signal_follower_supply,
            signal_follower_temperature);

        motor_master.optimizeBusUtilization();
        motor_follower.optimizeBusUtilization();
        
        if (Robot.isSimulation()) {
            sim_master = motor_master.getSimState();
            sim_master.setMotorType(MotorType.KrakenX60);
            sim_master.Orientation = ChassisReference.Clockwise_Positive;
            sim_follower = motor_follower.getSimState();
            sim_follower.setMotorType(MotorType.KrakenX60);
            sim_follower.Orientation = ChassisReference.CounterClockwise_Positive;
            sim_mechanism = new ElevatorSim(
                DCMotor.getKrakenX60(2), 
                ElevatorConstants.gearing, 
                Units.lbsToKilograms(5), 
                Units.inchesToMeters(ElevatorConstants.drum_circumference/(2.0 * Math.PI)), 
                0,
                Units.inchesToMeters(ElevatorConstants.mechanical_max_height),
                true, 
                0);
        }

        motor_follower.setControl(new Follower(motor_master.getDeviceID(), MotorAlignmentValue.Opposed));
    }

    @Override
    public void updateInputs(ElevatorData data) {
        if (Robot.isSimulation()) {
            // TODO: Fix this once CTRE Fixes their vendor dep
            sim_master.setSupplyVoltage(Volts.of(12.0)); // Fixes 2027 WPILib Alpha 8 Bug
            sim_follower.setSupplyVoltage(Volts.of(12.0));

            sim_mechanism.setInputVoltage(sim_master.getMotorVoltage());
            sim_mechanism.update(0.020);

            Telemetry.log("Elevator/Input Voltage", sim_master.getMotorVoltage());
            Telemetry.log("Elevator/Sim Mechanism Position", sim_mechanism.getPosition());
            Telemetry.log("Elevator/Sim Master/Duty Cycle", motor_master.getDutyCycle().getValueAsDouble());
            Telemetry.log("Elevator/Sim Master/Hit Stator", motor_master.getFault_StatorCurrLimit().getValue());
            Telemetry.log("Elevator/Sim Master/Hit Supply", motor_master.getFault_SupplyCurrLimit().getValue());
            
            sim_master.setRawRotorPosition(Units.metersToInches(sim_mechanism.getPosition()) * ElevatorConstants.drum_conversion_factor);
            sim_master.setRotorVelocity(Units.metersToInches(sim_mechanism.getVelocity()) * ElevatorConstants.drum_conversion_factor);
            sim_follower.setRawRotorPosition(Units.metersToInches(sim_mechanism.getPosition()) * ElevatorConstants.drum_conversion_factor);
            sim_follower.setRotorVelocity(Units.metersToInches(sim_mechanism.getVelocity()) * ElevatorConstants.drum_conversion_factor);
        }

        BaseStatusSignal.refreshAll(
            signal_master_position,
            signal_master_velocity,
            signal_master_acceleration,
            signal_master_voltage_output,
            signal_master_stator,
            signal_master_supply,
            signal_master_temperature, 
            signal_follower_position,
            signal_follower_velocity,
            signal_follower_acceleration,
            signal_follower_voltage_output,
            signal_follower_stator,
            signal_follower_supply,
            signal_follower_temperature
        );
        
        data.height = Inches.of(signal_master_position.getValue().in(Rotations));
        data.setpoint = Inches.of(control_position.getPositionMeasure().in(Rotations));
        data.motor_position = signal_master_position.getValue();
        data.motor_velocity = signal_master_velocity.getValue();
        data.motor_acceleration = signal_master_acceleration.getValue();

        data.left_voltage = signal_master_voltage_output.getValue();
        data.left_stator = signal_master_stator.getValue();
        data.left_supply = signal_master_supply.getValue();
        data.left_temperature = signal_master_temperature.getValue();

        data.right_voltage = signal_follower_voltage_output.getValue();
        data.right_stator = signal_follower_stator.getValue();
        data.right_supply = signal_follower_supply.getValue();
        data.right_temperature = signal_follower_temperature.getValue();
    }

    @Override
    public void set_height(Distance height) {
        motor_master.setControl(control_position.withPosition(height.in(Inches)));
    }

    @Override
    public void set_voltage(Voltage volts) {
        Telemetry.log("Elevator/Controls/Applied Voltage", volts);
        motor_master.setControl(control_voltage.withOutput(volts));
        inputVoltage = volts;
    }

    @Override
    public void updatePID(double kp, double ki, double kd, double ks, double kv, double kg) {
        configuration.Slot0.kP = kp;
        configuration.Slot0.kI = ki;
        configuration.Slot0.kD = kd;
        configuration.Slot0.kS = ks;
        configuration.Slot0.kV = kv;
        configuration.Slot0.kG = kg;

        motor_master.getConfigurator().apply(configuration, 0.25);
        motor_follower.getConfigurator().apply(configuration, 0.25);
    }
}