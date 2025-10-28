// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Commands.DomainExpansion;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.Commands.WristCommand;
import frc.robot.subsystems.claw.Wrist.WristState;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class HumanPlayerIntake extends SequentialCommandGroup {
  /** Creates a new HumanSeq. */
  private HumanPlayerIntake1 h = new HumanPlayerIntake1();
  private WristCommand w = new WristCommand(WristState.HUMANSTATIONINTAKE);
  private WaitCommand l = new WaitCommand(0.2);
  public HumanPlayerIntake() {
    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());
    
    addCommands( h);
  }
}
