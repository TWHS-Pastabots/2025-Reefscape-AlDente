package frc.robot.Commands.DomainExpansion;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Commands.PivotCommand;
import frc.robot.subsystems.pivot.Pivot.PivotState;
public class L4NTX extends SequentialCommandGroup{
    public L4NTX(){
        addCommands(new L4coraly(), new PivotCommand(PivotState.L4CORALSCORE));
    }
}
