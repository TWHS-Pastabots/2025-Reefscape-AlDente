// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Commands.DomainExpansion;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelDeadlineGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.Commands.WristCommand;
import frc.robot.subsystems.claw.Wrist.WristState;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class human1 extends ParallelDeadlineGroup {
  /** Creates a new human1. */
  public human1() {
    // Add the deadline command in the super() call. Add other commands using
    // addCommands().
    super(new WaitCommand(.35));
    // addCommands(new FooCommand(), new BarCommand());
    addCommands(new WristCommand(WristState.HUMANSTATIONINTAKE));
  }
}
