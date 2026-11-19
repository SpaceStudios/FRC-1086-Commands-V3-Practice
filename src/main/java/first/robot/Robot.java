// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package first.robot;

import static org.wpilib.units.Units.Inches;
import static org.wpilib.units.Units.Volts;

import org.wpilib.command3.Scheduler;
import org.wpilib.command3.button.CommandXboxController;
import org.wpilib.epilogue.Epilogue;
import org.wpilib.epilogue.Logged;
import org.wpilib.epilogue.Logged.Strategy;
import org.wpilib.framework.OpModeRobot;

import first.robot.mechanisms.elevator.Elevator;
import first.robot.mechanisms.elevator.ElevatorIOTalonFX;

@Logged( strategy = Strategy.OPT_IN )
public class Robot extends OpModeRobot {
  private static Robot instance;
  @Logged(name = "Elevator")
  public final Elevator elevator = new Elevator(new ElevatorIOTalonFX());
  private final CommandXboxController controller_driver = new CommandXboxController(0);
  public Robot() {
    controller_driver.y().onTrue(elevator.set_position(Inches.of(36.0)));
    controller_driver.rightTrigger().whileTrue(elevator.set_voltage(Volts.of(4.0)));
  }

  @Override
  public void robotPeriodic() {
    Scheduler.getDefault().run();
    Epilogue.update(this);
  }
}
