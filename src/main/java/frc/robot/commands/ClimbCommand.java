package frc.robot.commands;

import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.Flags;
import frc.robot.controllers.AbstractController;
import frc.robot.subsystems.climber.ClimberSubsystem;
import frc.robot.subsystems.intake.IntakeSubsystem;
import frc.robot.subsystems.shooter.ShooterSubsystem;
import frc.robot.subsystems.staticsubsystems.RobotGyro;
import frc.robot.subsystems.swerve.DriveTrainSubsystem;
import frc.robot.util.Util;

public class ClimbCommand extends Command {
//    private final DriveTrainSubsystem driveTrain;
//    private final AbstractController joystick;

    private final ShooterSubsystem shooter;
    private final IntakeSubsystem intake;
    private final ClimberSubsystem climber;

    public static final double MAX_SPEED_METERS_PER_SEC = Flags.DriveTrain.LOWER_MAX_SPEED ? 1.5 : 3;

    public ClimbCommand(
            ShooterSubsystem shooter,
            IntakeSubsystem intake,
            ClimberSubsystem climber
    ) {
        this.shooter = shooter;
        this.intake = intake;
        this.climber = climber;

        addRequirements(shooter, intake, climber);
    }

    @Override
    public void initialize() {}

    public double vedanthsMagicalRadiansToVoltageFunction(double x) {
        double k = 216.0 / 5.0 / Math.pow(Math.PI, 3);
        return k * Math.pow(x, 3);
    }

    double[] correctRoll() {
        // returns [leftMotorSpeed, rightMotorSpeed]
        // if the roll is a POSITIVE value, that means the LEFT SIDE is TOO HIGH
        // like this -> \ (left side of backslash is UP)
        // so we send voltage to the LEFT MOTOR

        double rollRadians = RobotGyro.getGyroAngleDegreesRoll() * Math.PI / 180;

        double finalVoltage;
        if (Math.abs(rollRadians) < Math.PI / 180) {
            // we don't need to correct cus everything is going great :)
            finalVoltage = 0.0;
        } else if (Math.abs(rollRadians) < Math.PI / 6) {
            // cool model function that gets the right amount of voltage
            finalVoltage = vedanthsMagicalRadiansToVoltageFunction(rollRadians);
        } else {
            // let nature take its course, robot is screwed
            finalVoltage = 0.0;
        }

        // adjust height, always going upwards
        if (finalVoltage > 0) {
            return new double[] {Math.abs(finalVoltage), 0};
        } else {
            return new double[] {0, Math.abs(finalVoltage)};
        }
    }

    void setClimberSpeeds() {
        double baseSpeed = 0.5;
        double[] climberSpeedErrors = correctRoll();
        double leftMotorSpeed = baseSpeed + climberSpeedErrors[0];
        double rightMotorSpeed = baseSpeed + climberSpeedErrors[1];
        climber.setRightMotorSpeed.accept(rightMotorSpeed);
        climber.setLeftMotorSpeed.accept(leftMotorSpeed);
    }

    @Override
    public void execute() {
        setClimberSpeeds();

        // fully extend intake & shooter
        double pitch = RobotGyro.getGyroAngleDegreesPitch(); // if the gyro is sideways just change this so it is actual pitch

        double intakePosition  = this.intake .getPivotPosition();
        double shooterPosition = this.shooter.getPivotPosition();
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
