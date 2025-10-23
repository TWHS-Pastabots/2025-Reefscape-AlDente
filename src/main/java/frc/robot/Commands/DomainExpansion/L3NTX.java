package frc.robot.Commands.DomainExpansion;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Commands.PivotCommand;
import frc.robot.subsystems.pivot.Pivot.PivotState;
public class L3NTX extends SequentialCommandGroup{
    public L3NTX(){
        addCommands(new L3coraly(), new PivotCommand(PivotState.L3CORALSCORE));
    }
}
