package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.controllers.AbstractController;
import frc.robot.subsystems.swerve.DriveTrainSubsystem;
import frc.robot.subsystems.swerve.SwerveModule;
import frc.robot.util.Util;
import edu.wpi.first.wpilibj.Timer;

public class IvanCommand extends Command {
    private final DriveTrainSubsystem driveTrain;
    private int i = 0;
    private Timer timer;

    public IvanCommand(DriveTrainSubsystem driveTrain) {
        this.driveTrain = driveTrain;

        addRequirements(driveTrain);
    }

    @Override
    public void initialize() {
        timer = new Timer();
        timer.start();
    }

    @Override
    public void execute() {
        if (timer.hasElapsed(3)) this.driveTrain.drive(0, -0.2, 0, false);
        else if (timer.hasElapsed(2)) this.driveTrain.drive(-0.2, 0, 0, false);
        else if (timer.hasElapsed(1)) this.driveTrain.drive(0, 0.2, 0, false);
        else this.driveTrain.drive(0.2, 0, 0, false);
    }

    // Called once the command ends or is interrupted.
    @Override
    public void end(boolean interrupted) {
        this.driveTrain.stop();
    }

    // Returns true when the command should end.
    @Override
    public boolean isFinished() {
        return timer.hasElapsed(4);
    }
}