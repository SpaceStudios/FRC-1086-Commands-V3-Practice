// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package first.robot.opmodes;

import static org.wpilib.units.Units.Inches;
import static org.wpilib.units.Units.Volts;

import org.wpilib.command3.button.CommandXboxController;
import org.wpilib.opmode.PeriodicOpMode;
import org.wpilib.opmode.Teleop;

import first.robot.Robot;

/** Add your docs here. */
@Teleop( name = "Teleoperated Default")
public class TeleOpDefault extends PeriodicOpMode {
    private final CommandXboxController controller_driver = new CommandXboxController(0);

    public TeleOpDefault() {}

    @Override
    public void start() {
        // Bind Buttons in Here
        // controller_driver.y().onTrue(Robot.elevator.set_position(Inches.of(36.0)));
        // controller_driver.rightTrigger().whileTrue(Robot.elevator.set_voltage(Volts.of(4.0)));
    }
}
