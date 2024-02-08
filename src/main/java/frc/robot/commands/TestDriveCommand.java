package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Flags;
import frc.robot.controllers.AbstractController;
import frc.robot.subsystems.swerve.DriveTrainSubsystem;
import frc.robot.subsystems.swerve.SwerveModule;
import frc.robot.util.RobotMathUtil;

public class TestDriveCommand extends Command {
    private final DriveTrainSubsystem driveTrain;
    private final AbstractController joystick;

    public static final double MAX_SPEED_METERS_PER_SEC = Flags.DriveTrain.LOWER_MAX_SPEED ? 3 : 5;

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
            System.out.println("name: " + module.getName() + ", abs: " + RobotMathUtil.roundNearestHundredth(module.getTurningAbsEncoderPositionConverted()) + ", rel: " + RobotMathUtil.roundNearestHundredth(module.getRelativeTurnRotations()));
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
