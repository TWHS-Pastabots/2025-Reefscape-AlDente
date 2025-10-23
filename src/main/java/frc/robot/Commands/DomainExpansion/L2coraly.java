package frc.robot.Commands.DomainExpansion;

import edu.wpi.first.wpilibj2.command.ParallelDeadlineGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.Commands.ElevatorCommand;
import frc.robot.Commands.PivotCommand;
import frc.robot.Commands.WristCommand;
import frc.robot.subsystems.claw.Wrist.WristState;
import frc.robot.subsystems.elevator.Elevator.ElevatorState;
import frc.robot.subsystems.pivot.Pivot.PivotState;

public class L2coraly extends ParallelDeadlineGroup{
    public L2coraly(){
        super(new WaitCommand(.2) );

        addCommands(new WristCommand(WristState.L2CORALSCORE), new ElevatorCommand(ElevatorState.L2CORALSCORE) );
    }
}
