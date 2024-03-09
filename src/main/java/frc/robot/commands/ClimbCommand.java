package frc.robot.commands;

import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Flags;
import frc.robot.controllers.AbstractController;
import frc.robot.subsystems.climber.ClimberSubsystem;
import frc.robot.subsystems.intake.IntakeSubsytem;
import frc.robot.subsystems.shooter.ShooterSubsystem;
import frc.robot.subsystems.staticsubsystems.RobotGyro;
import frc.robot.subsystems.swerve.DriveTrainSubsystem;
import frc.robot.util.Util;

public class ClimbCommand extends Command {
//    private final DriveTrainSubsystem driveTrain;
//    private final AbstractController joystick;

    private final ShooterSubsystem shooter;
    private final IntakeSubsytem intake;
    private final ClimberSubsystem climber;

    public static final double MAX_SPEED_METERS_PER_SEC = Flags.DriveTrain.LOWER_MAX_SPEED ? 1.5 : 3;

    public ClimbCommand(
            ShooterSubsystem shooter,
            IntakeSubsytem intake,
            ClimberSubsystem climber
    ) {
        this.shooter = shooter;
        this.intake = intake;
        this.climber = climber;

        addRequirements(shooter, intake, climber);
    }

    @Override
    public void initialize() {
    }

    @Override
    public void execute() {
        // TODO
        double pitch = RobotGyro.getGyroAngleDegreesPitch(); // if the gyro is sideways just change this so it is actual pitch

        // Bad design, temporary unless it works
        if (pitch > 10) {
            shooter.setBottomMotorSpeed(0.2);
            intake.setPivotSpeed(-0.2);
        } else if (pitch < 10 && pitch > -10) {
            shooter.setBottomMotorSpeed(0);
            intake.setPivotSpeed(0);
        } else {
            shooter.setBottomMotorSpeed(-0.2);
            intake.setPivotSpeed(0.2);
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
