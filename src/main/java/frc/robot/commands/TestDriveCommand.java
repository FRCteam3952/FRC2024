package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.controllers.AbstractController;
import frc.robot.subsystems.swerve.DriveTrainSubsystem;
import frc.robot.subsystems.swerve.SwerveModule;
import frc.robot.util.Util;

public class TestDriveCommand extends Command {
    private final DriveTrainSubsystem driveTrain;
    private final AbstractController joystick;

    public TestDriveCommand(DriveTrainSubsystem driveTrain, AbstractController joystick) {
        this.driveTrain = driveTrain;
        this.joystick = joystick;

        addRequirements(driveTrain);
    }

    @Override
    public void initialize() {
    }

    @Override
    public void execute() {
        this.driveTrain.directDriveSpeed(this.joystick.getLeftVerticalMovement());
        this.driveTrain.directTurnSpeed(this.joystick.getRightHorizontalMovement());

        for(SwerveModule module : this.driveTrain.swerveModules) {
            System.out.println("name: " + module.getName() + ", abs abs: " + Util.nearestHundredth(module.getTurnAbsolutelyAbsolutePosition()) + ", abs: " + Util.nearestHundredth(module.getTurnAbsEncoderPosition()) + ", rel: " + Util.nearestHundredth(module.getTurnRelativePosition()));
        }
    }

    // Called once the command ends or is interrupted.
    @Override
    public void end(boolean interrupted) {

    }

    // Returns true when the command should end.
    @Override
    public boolean isFinished() {
        return false;
    }
}
