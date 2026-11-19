// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package first.robot.mechanisms.elevator;

import static org.wpilib.units.Units.Inches;
import static org.wpilib.units.Units.Volts;

import org.wpilib.command3.Command;
import org.wpilib.command3.Mechanism;
import org.wpilib.epilogue.Logged;
import org.wpilib.math.util.MathUtil;
import org.wpilib.units.measure.Distance;
import org.wpilib.units.measure.Voltage;

import first.robot.mechanisms.elevator.ElevatorIO.ElevatorData;

/** Add your docs here. */
public class Elevator implements Mechanism {
    private final ElevatorIO io;
    private final ElevatorData data = new ElevatorData();
    public Elevator(ElevatorIO io) {
        this.io = io;
    }

    @Logged
    public ElevatorData updateInputs() {
        io.updateInputs(data);
        if (
        ElevatorConstants.controls_tunable_kp.hasChanged() ||
        ElevatorConstants.controls_tunable_ki.hasChanged() ||
        ElevatorConstants.controls_tunable_kd.hasChanged() ||
        ElevatorConstants.controls_tunable_ks.hasChanged() ||
        ElevatorConstants.controls_tunable_kv.hasChanged() ||
        ElevatorConstants.controls_tunable_kg.hasChanged()) {
            io.updatePID(
                ElevatorConstants.controls_tunable_kp.get(), 
                ElevatorConstants.controls_tunable_ki.get(), 
                ElevatorConstants.controls_tunable_kd.get(),
                ElevatorConstants.controls_tunable_ks.get(),
                ElevatorConstants.controls_tunable_kv.get(),
                ElevatorConstants.controls_tunable_kg.get());
        }
        return data;
    }

    public Command set_position(Distance height) {
        return run(coroutine -> {
            io.set_height(height);
        }).named("Set Elevator Height");
    }

    public Command set_voltage(Voltage volts) {
        return runRepeatedly(() -> {
            io.set_voltage(volts);
        }).whenCanceled(() -> {
            io.set_voltage(Volts.zero());
        }).named("Set Voltage");
    }

    // Checks if the position is near the setpoint and within a 1/16th of an inch tolerance
    @Logged
    public boolean at_setpoint() {
        return MathUtil.isNear(data.setpoint.in(Inches), data.height.in(Inches), 1.0/16.0);
    }
}
