package frc.robot.commands.drivetrain;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.ShooterConstants;
import frc.robot.subsystems.ShooterSys;
import edu.wpi.first.wpilibj.Timer;

public class AutoShootCmd extends Command {
        private final ShooterSys shooterSys;
        private final Timer timer;
        private final double duration = 5.0; // seconds

    public AutoShootCmd(ShooterSys shooterSys) {
        this.shooterSys = shooterSys;
        timer = new Timer();
    }

    // Called when the command is initially scheduled.
    @Override
    public void initialize() {
        timer.reset();
        timer.start();
    }

    // Called every time the scheduler runs while the command is scheduled.
    @Override
    public void execute() {
        shooterSys.setShooterRPM(ShooterConstants.shooterRPM); // Set to desired RPM
        System.out.println(shooterSys.getShooterRPM());
    }

    // Called once the command ends or is interrupted.
    @Override
    public void end(boolean interrupted) {
        timer.stop();
        shooterSys.stop();
    }

    // Returns true when the command should end.
    @Override
    public boolean isFinished() {
        return timer.hasElapsed(duration);
    }
}
