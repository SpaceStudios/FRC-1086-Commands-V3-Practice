// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package first.robot.mechanisms.elevator;

import static org.wpilib.units.Units.Amps;
import static org.wpilib.units.Units.Inches;
import static org.wpilib.units.Units.Millimeters;

import java.util.function.DoubleSupplier;

import org.wpilib.telemetry.Telemetry;
import org.wpilib.tunable.Tunable;
import org.wpilib.tunable.TunableDouble;
import org.wpilib.tunable.Tunables;
import org.wpilib.units.measure.Current;
import org.wpilib.units.measure.Distance;

import com.ctre.phoenix6.signals.InvertedValue;

/** Add your docs here. */
public class ElevatorConstants {
    public static final Current motor_supply_limit = Amps.of(60);
    public static final Current motor_stator_limit = Amps.of(80);
    public static final int drum_pulley_teeth = 36;
    public static final Distance drum_pulley_pitch = Millimeters.of(5);
    public static final double gearing = 5.0;
    public static final double drum_circumference = ((double) drum_pulley_teeth) * drum_pulley_pitch.in(Inches);
    public static final double drum_conversion_factor = gearing / (drum_circumference);
    public static final double mechanical_max_height = 72.0; // 72 Inches
    public static final InvertedValue controls_positive_direction = InvertedValue.Clockwise_Positive;
    public static final double controls_pid_kp = 50.0;
    public static final double controls_pid_ki = 0.0;
    public static final double controls_pid_kd = 0.0;
    public static final double controls_pid_ks = 0.0;
    public static final double controls_pid_kv = 0.0;
    public static final double controls_pid_kg = 0.0;

    public static final TunableDouble controls_tunable_kp = Tunables.addDouble("Elevator/kP",controls_pid_kp);
    public static final TunableDouble controls_tunable_ki = Tunables.addDouble("Elevator/kI",controls_pid_ki);
    public static final TunableDouble controls_tunable_kd = Tunables.addDouble("Elevator/kD",controls_pid_kd);
    public static final TunableDouble controls_tunable_ks = Tunables.addDouble("Elevator/kS",controls_pid_ks);
    public static final TunableDouble controls_tunable_kv = Tunables.addDouble("Elevator/kV",controls_pid_kv);
    public static final TunableDouble controls_tunable_kg = Tunables.addDouble("Elevator/kG",controls_pid_kg);
}