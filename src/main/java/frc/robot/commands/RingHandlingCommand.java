package frc.robot.commands;

import edu.wpi.first.networktables.DoublePublisher;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.Constants.OperatorConstants.ControllerConstants;
import frc.robot.controllers.AbstractController;
import frc.robot.subsystems.conveyor.ConveyorSubsystem;
import frc.robot.subsystems.intake.IntakeSubsystem;
import frc.robot.subsystems.shooter.ShooterSubsystem;
import frc.robot.subsystems.staticsubsystems.ColorSensor;
import frc.robot.util.ControlHandler;
import frc.robot.util.NetworkTablesUtil;

/**
 * also known as SonicTheHedgehogCommand
 */
public class RingHandlingCommand extends Command {
    private static final DoublePublisher rpmPub = NetworkTablesUtil.MAIN_ROBOT_TABLE.getDoubleTopic("shooter_rpm").publish();
    private final ShooterSubsystem shooter;
    private final IntakeSubsystem intake;
    private final ConveyorSubsystem conveyor;
    private final AbstractController joystick;
    private boolean intakeToggledOn = false;
    private boolean hasHandledNote = false;
    private int reverseTimerElapsed = 0;
    private boolean shouldReverse = false;
    private boolean intakeUp = false;

    public RingHandlingCommand(ShooterSubsystem shooter, IntakeSubsystem intake, ConveyorSubsystem conveyor, AbstractController joystick) {
        this.shooter = shooter;
        this.intake = intake;
        this.conveyor = conveyor;
        this.joystick = joystick;

        addRequirements(intake);
    }

    @Override
    public void initialize() {
    }

    @Override
    public void execute() {
        if (joystick.leftShoulderButton().getAsBoolean()) {
            this.shooter.pivotToAngle(54);
        } else if (joystick.leftShoulderTrigger().getAsBoolean()) {
            this.shooter.pivotToAngle(30);
        }

        ControlHandler.get(joystick, ControllerConstants.INTAKE_RUN)
            .onTrue(new InstantCommand(() -> this.intakeToggledOn = true))
            .onFalse(new InstantCommand(() -> this.intakeToggledOn = false));
        
        ControlHandler.get(joystick, ControllerConstants.INTAKE_POS_TOGGLE)
            .onTrue(new InstantCommand(() -> {
                intakeUp = !intakeUp;
                if(intakeUp) {
                    double throughboreValue = this.intake.getThroughboreEncoder().getAbsoluteEncoderValue();
                    if (throughboreValue > 0 && throughboreValue < 3) {
                        this.intake.pivotToAngle(70 + throughboreValue);
                    } else {
                        this.intake.pivotToAngle(70);
                    }
                } else {
                    this.intake.pivotToAngle(0);
                }
            }));

        if (shouldReverse && reverseTimerElapsed++ < 1) {
            System.out.println("reversing at " + reverseTimerElapsed);
            this.intake.setIntakeSpeed(-0.2, -0.2);
            this.conveyor.setConveyorMotorsSpeed(0.2);
            this.conveyor.setShooterFeederMotorSpeed(-0.6);
        } else {
            shouldReverse = false;
            reverseTimerElapsed = 0;
        }

        if (ControlHandler.get(joystick, ControllerConstants.INTAKE_REVERSE).getAsBoolean()) { // eject takes priority
            shouldReverse = true;
            // this.shooter.setBottomMotorSpeed(-0.3);
        } else if (ColorSensor.isNoteColor() && !hasHandledNote) {
            shouldReverse = true;
            intakeToggledOn = false;
        } else if (intakeToggledOn) {
            this.intake.setIntakeSpeed(1, 1);
            this.conveyor.setConveyorMotorsSpeed(-1);
            this.conveyor.setShooterFeederMotorSpeed(1);
            // this.shooter.setBottomMotorSpeed(0);
        } else if (!shouldReverse) {
            this.intake.setIntakeSpeed(0, 0);
            this.conveyor.setConveyorMotorsSpeed(0);
            this.conveyor.setShooterFeederMotorSpeed(0);
            // this.shooter.setBottomMotorSpeed(0);
        }

        // when the shooter is up high enough we GO BRRRR
        if (ControlHandler.get(this.joystick, ControllerConstants.SHOOTER_RUN).getAsBoolean()) {
            shooter.setMotorRpm(600);
            if (shooter.getShooterRpm() > 500) {
                this.conveyor.setShooterFeederMotorSpeed(1);
                this.conveyor.setConveyorMotorsSpeed(-0.3);
                hasHandledNote = false;
            }
        } else {
            shooter.stopShooterPID();
        }
        // System.out.println("RPM: " + shooter.getShooterRpm());
        rpmPub.set(shooter.getShooterRpm());

        // this.intake.setPivotSpeed(-joystick.getRightVerticalMovement());

        // System.out.println("intake position: " + Util.nearestHundredth(intake.getPivotPosition()));

        // System.out.println("lower intake current: " + this.intake.getFollowerMotorCurrent() + ", top current: " + this.intake.getLeaderMotorCurrent());
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
