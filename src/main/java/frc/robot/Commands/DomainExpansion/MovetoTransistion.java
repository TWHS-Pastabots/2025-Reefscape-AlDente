// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Commands.DomainExpansion;


import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Commands.ElevatorCommand;
import frc.robot.Commands.PivotCommand;
import frc.robot.Commands.WristCommand;
import frc.robot.subsystems.claw.Claw;
import frc.robot.subsystems.claw.Wrist.WristState;
import frc.robot.subsystems.elevator.Elevator.ElevatorState;
import frc.robot.subsystems.pivot.Pivot.PivotState;
import frc.robot.Commands.DomainExpansion.*;
/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class MovetoTransistion extends Command {
  private WristCommand wrist;
  private PivotCommand pivot;
  private ElevatorCommand elevator;
  private boolean ended;
  private boolean transitionReady;
  private Claw claw;
  private double timer;
  private Transition transition;
  private HumanPlayerIntake humanPlayerIntake;
  /** Creates a new GroundIntakeCoral. */
  public MovetoTransistion() {
    // wrist = new WristCommand(WristState.TRANSITIONSTATE);
    // pivot = new PivotCommand(PivotState.TRANSITIONSTATE);
    // elevator = new ElevatorCommand(ElevatorState.AUTONTRANSITION);
    
    claw = Claw.getInstance();
    transitionReady = false;
    transition = new Transition();
    humanPlayerIntake = new HumanPlayerIntake();
    timer = 0;
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    transition.initialize();
    humanPlayerIntake.initialize();
    timer = Timer.getFPGATimestamp();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {

      transition.schedule();
      
      if(Timer.getFPGATimestamp() > timer + 1.4  ){
        transition.cancel();
        humanPlayerIntake.schedule();
        
            if(Timer.getFPGATimestamp() > timer +2.5){
                humanPlayerIntake.cancel();
                ended = true;
                
            }
      }
      
      // if(pivot.isFinished() && wrist.isFinished()){
      //   claw.clawReverse(.6);
      // }
      
    }

  @Override
  public void end(boolean interrupted) {
  //  claw.clawOff();
//   wrist.setLastState();
  humanPlayerIntake.cancel();
  }

  // Returns true when the command should end.
  @Override
   public boolean isFinished() {
    return ended;
  }
}
