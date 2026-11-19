// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package first.robot.mechanisms.elevator;

import org.wpilib.hardware.bus.CANPort;

import com.ctre.phoenix6.CANBus;

/** Add your docs here. */
public class ElevatorMap {
    public static final CANBus elevator_bus = new CANBus(CANPort.CAN_S2);
    public static final int elevator_master = 1;
    public static final int elevator_follower = 2;
}
