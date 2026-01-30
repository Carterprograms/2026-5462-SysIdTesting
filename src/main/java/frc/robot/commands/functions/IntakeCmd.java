package frc.robot.commands.functions;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.IntakeSys;
import frc.robot.Constants.RollerConstants;

public class IntakeCmd extends Command {

    private final IntakeSys intakeSys;
    
    public IntakeCmd(IntakeSys intakeSys) {
        this.intakeSys = intakeSys;
    }

    @Override
    public void initialize() {}

    @Override
    public void execute() {
        System.out.println(intakeSys.getRollerRPM());

        intakeSys.setRollerRPM(RollerConstants.rollerRPM);
    }

    @Override
    public void end(boolean interrupted) {
        intakeSys.stop();
    }

    @Override
    public boolean isFinished() {
        return false;
    }
}