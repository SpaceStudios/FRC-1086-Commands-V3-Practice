// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package first.robot.mechanisms.elevator;

import static org.wpilib.units.Units.Amps;
import static org.wpilib.units.Units.Celsius;
import static org.wpilib.units.Units.Inches;
import static org.wpilib.units.Units.Radians;
import static org.wpilib.units.Units.RadiansPerSecond;
import static org.wpilib.units.Units.RadiansPerSecondPerSecond;
import static org.wpilib.units.Units.Volts;

import org.wpilib.epilogue.Logged;
import org.wpilib.units.measure.Angle;
import org.wpilib.units.measure.AngularAcceleration;
import org.wpilib.units.measure.AngularVelocity;
import org.wpilib.units.measure.Current;
import org.wpilib.units.measure.Distance;
import org.wpilib.units.measure.Temperature;
import org.wpilib.units.measure.Voltage;

/** Add your docs here. */
public interface ElevatorIO {
    @Logged
    public class ElevatorData {
        public Distance setpoint = Inches.zero();
        public Distance height = Inches.zero();

        public Angle motor_position = Radians.zero();
        public AngularVelocity motor_velocity = RadiansPerSecond.zero();
        public AngularAcceleration motor_acceleration = RadiansPerSecondPerSecond.zero();

        public Voltage left_voltage = Volts.zero();
        public Current left_stator = Amps.zero();
        public Current left_supply = Amps.zero();
        public Temperature left_temperature = Celsius.zero();

        public Voltage right_voltage = Volts.zero();
        public Current right_stator = Amps.zero();
        public Current right_supply = Amps.zero();
        public Temperature right_temperature = Celsius.zero();
    }

    public default void updateInputs(ElevatorData data) {}
    public default void set_height(Distance height) {}
    public default void set_voltage(Voltage voltage) {}
    public default void updatePID(double kp, double ki, double kd, double ks, double kv, double kg) {}
}