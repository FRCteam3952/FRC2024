package frc.robot.commands;

import edu.wpi.first.networktables.DoublePublisher;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.Constants.OperatorConstants.ControllerConstants;
import frc.robot.controllers.AbstractController;
import frc.robot.controllers.FlightJoystick;
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
    private final AbstractController primaryController;
    private final FlightJoystick sideJoystick;

    private boolean intakeToggledOn = false;
    private boolean hasHandledNote = false;
    private int reverseTimerElapsed = 0;
    private boolean shouldReverse = false;
    private boolean intakeUp = true;
    private boolean hasNote = false;

    private final InstantCommand toggleIntakeRun = new InstantCommand(() -> intakeToggledOn = !intakeToggledOn);
    private final InstantCommand toggleIntakePos = new InstantCommand(() -> intakeUp = !intakeUp);

    private final Trigger reverseIntake, runShooterHigh, runShooterAmp;

    public RingHandlingCommand(ShooterSubsystem shooter, IntakeSubsystem intake, ConveyorSubsystem conveyor, AbstractController primaryController, FlightJoystick sideJoystick) {
        this.shooter = shooter;
        this.intake = intake;
        this.conveyor = conveyor;
        this.primaryController = primaryController;
        this.sideJoystick = sideJoystick;

        this.reverseIntake = ControlHandler.get(primaryController, ControllerConstants.INTAKE_REVERSE);
        this.runShooterHigh = ControlHandler.get(primaryController, ControllerConstants.SHOOTER_RUN_HIGH_SPEED);
        this.runShooterAmp = ControlHandler.get(primaryController, ControllerConstants.SHOOTER_RUN_AMP_SPEED);

        addRequirements(shooter, intake, conveyor);
    }

    @Override
    public void initialize() {
        ControlHandler.get(primaryController, ControllerConstants.INTAKE_RUN)
            .onTrue(toggleIntakeRun);
        
        ControlHandler.get(primaryController, ControllerConstants.INTAKE_POS_TOGGLE)
            .onTrue(toggleIntakePos);
    }

    private double shooterAngle = 45;

    private double flapAngleTargetL = 155;
    private double flapAngleTargetR = 20;
    @Override
    public void execute() {
        // this.shooter.flapToAngle(90 * (primaryController.getRightVerticalMovement() + 1));
        if(sideJoystick.getRawButtonPressedWrapper(7)) {
            flapAngleTargetL = 155;
            flapAngleTargetR = 20;
        } else if(sideJoystick.getRawButtonPressedWrapper(6)) {
            flapAngleTargetL = 60;
            flapAngleTargetR = 115;
        } else if(sideJoystick.getRawButtonPressedWrapper(5)) {
            flapAngleTargetL = 45;
            flapAngleTargetR = 130;
        }

        this.shooter.flapToAngle(flapAngleTargetL, flapAngleTargetR);

        /*
        if (joystick.leftShoulderButton().getAsBoolean()) {
            this.shooter.pivotToAngle(54);
        } else if (joystick.leftShoulderTrigger().getAsBoolean()) {
            this.shooter.pivotToAngle(30);
        }*/

        if(primaryController.getPOV() == 0) {
            shooterAngle++;
        } else if(primaryController.getPOV() == 180) {
            shooterAngle--;
        }

        this.shooter.pivotToAngle(shooterAngle);

        if(intakeUp) {
            double throughboreValue = this.intake.getThroughboreEncoder().getAbsoluteEncoderValue();
            if (throughboreValue > 0 && throughboreValue < 3) {
                this.intake.pivotToAngle(73 + throughboreValue);
            } else {
                this.intake.pivotToAngle(73); // 73
            }
        } else {
            this.intake.pivotToAngle(0);
        }

        if (shouldReverse && reverseTimerElapsed++ < 1) {
            // System.out.println("reversing at " + reverseTimerElapsed + ", note handled: " + hasHandledNote + ", does it have one: " + hasNote);
            this.intake.setIntakeSpeed(-0.2, -0.2);
            this.conveyor.setConveyorMotorsSpeed(0.2);
            this.conveyor.setShooterFeederMotorSpeed(-0.3);
        } else {
            shouldReverse = false;
            reverseTimerElapsed = 0;
            if(hasNote) {
                hasHandledNote = true;
                intakeUp = true;
            }
        }
    
        // System.out.println("note handled? " + hasHandledNote);

        if (reverseIntake.getAsBoolean()) { // eject takes priority
            shouldReverse = true;
        } else if (ColorSensor.isNoteColor()) {
            // System.out.println("NOTE FOUND");
            if(!hasHandledNote) {
                shouldReverse = true;
                intakeToggledOn = false;
                hasNote = true;
            }
        }
        
        if (intakeToggledOn) {
            if(intakeUp) {
                intakeUp = false;
            }
            this.intake.setIntakeSpeed(0.75, 1);
            this.conveyor.setConveyorMotorsSpeed(-0.7);
            this.conveyor.setShooterFeederMotorSpeed(0.7);
        } else if (!shouldReverse) {
            this.intake.setIntakeSpeed(0, 0);
            this.conveyor.setConveyorMotorsSpeed(0);
            this.conveyor.setShooterFeederMotorSpeed(0);
        }

        // when the shooter is up high enough we GO BRRRR
        if (runShooterHigh.getAsBoolean()) {
            shooter.setMotorRpm(2700);
            if (shooter.getShooterRpm() > 2600) {
                this.conveyor.setShooterFeederMotorSpeed(1);
                this.conveyor.setConveyorMotorsSpeed(-0.7);
                hasHandledNote = false;
                hasNote = false;
            }
        } else if(runShooterAmp.getAsBoolean()) {
            shooter.setMotorRpm(1400);
            if (shooter.getShooterRpm() > 1300) {
                this.conveyor.setShooterFeederMotorSpeed(1);
                this.conveyor.setConveyorMotorsSpeed(-0.7);
                hasHandledNote = false;
                hasNote = false;
            }
        } else {
            shooter.setMotorRpm(0); // 1300
        }
        // System.out.println("RPM: " + shooter.getShooterRpm());
        // rpmPub.set(shooter.getShooterRpm());

        // this.intake.setPivotSpeed(-joystick.getRightVerticalMovement());

        // System.out.println("intake position: " + Util.nearestHundredth(intake.getPivotPosition()));

        // System.out.println("lower intake current: " + this.intake.getFollowerMotorCurrent() + ", top current: " + this.intake.getLeaderMotorCurrent());
    }

    private double autoAimAngle() {
        // the middle of the speaker is 6'8 from the ground (avg. of bottom 6'6 and top 6'10+7/8), our shooter pivot is 16.5 in off the ground.

        return 0;
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
