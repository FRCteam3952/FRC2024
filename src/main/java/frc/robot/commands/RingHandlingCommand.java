package frc.robot.commands;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.networktables.DoublePublisher;
import edu.wpi.first.networktables.GenericPublisher;
import edu.wpi.first.networktables.NetworkTableType;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.Constants.OperatorConstants.ControllerConstants;
import frc.robot.controllers.AbstractController;
import frc.robot.subsystems.conveyor.ConveyorSubsystem;
import frc.robot.subsystems.intake.IntakeSubsystem;
import frc.robot.subsystems.shooter.ShooterSubsystem;
import frc.robot.subsystems.staticsubsystems.ColorSensor;
import frc.robot.util.AprilTagHandler;
import frc.robot.util.ControlHandler;
import frc.robot.util.NetworkTablesUtil;
import frc.robot.util.Util;

import java.util.Optional;

/**
 * also known as SonicTheHedgehogCommand
 */
public class RingHandlingCommand extends Command {
    private static final DoublePublisher rpmPub = NetworkTablesUtil.MAIN_ROBOT_TABLE.getDoubleTopic("shooter_rpm").publish();
    private static final GenericPublisher hasNotePub = NetworkTablesUtil.getPublisher("robot", "hasNote", NetworkTableType.kBoolean);
    private static final GenericPublisher hasHandledNotePub = NetworkTablesUtil.getPublisher("robot", "hasNote", NetworkTableType.kBoolean);
    private static final GenericPublisher hasAutoAimAprilTag = NetworkTablesUtil.getPublisher("robot", "hasAutoAimAprilTag", NetworkTableType.kBoolean);

    private final ShooterSubsystem shooter;
    private final IntakeSubsystem intake;
    private final ConveyorSubsystem conveyor;
    private final AbstractController primaryController;
    private final AbstractController secondaryController;
    private final AprilTagHandler aprilTagHandler;

    private boolean intakeToggledOn = false;
    private boolean hasHandledNote = false;
    private int reverseTimerElapsed = 0;
    private boolean shouldReverse = false;
    private boolean intakeUp = false;
    private boolean hasNote = false;
    private boolean idleShooterRpm = false;

    private enum FlapStage {
        DOWN,
        MIDDLE,
        UP
    }

    private FlapStage flapStage = FlapStage.DOWN;

    private final InstantCommand toggleIntakeRun = new InstantCommand(() -> {
        if(!hasNote) {
            intakeToggledOn = !intakeToggledOn;
        }
    });
    private final InstantCommand toggleIntakePos = new InstantCommand(() -> intakeUp = !intakeUp);
    private final InstantCommand toggleIdleShooterRpm = new InstantCommand(() -> idleShooterRpm = !idleShooterRpm);

    private final InstantCommand toggleFlapStage = new InstantCommand(() -> {
        if(this.flapStage == FlapStage.DOWN) {
            this.flapStage = FlapStage.MIDDLE;
        } else {
            this.flapStage = FlapStage.DOWN;
        }
    });

    private final InstantCommand wiggleFlap = new InstantCommand(() -> {
        if(this.flapStage == FlapStage.MIDDLE) {
            this.flapStage = FlapStage.UP;
        } else if(this.flapStage == FlapStage.UP) {
            this.flapStage = FlapStage.MIDDLE;
        }
    });

    private final InstantCommand resetState = new InstantCommand(() -> {
        hasHandledNote = false;
        shouldReverse = false;
        reverseTimerElapsed = 0;
        hasNote = false;
    });

    private final Trigger reverseIntake, runShooterHigh, runShooterAmp, autoAimSubwoofer;

    // 14ft = 4.267m => 2700 rpm
    // linear
    public RingHandlingCommand(ShooterSubsystem shooter, IntakeSubsystem intake, ConveyorSubsystem conveyor, AbstractController primaryController, AbstractController secondaryController, AprilTagHandler aprilTagHandler) {
        this.shooter = shooter;
        this.intake = intake;
        this.conveyor = conveyor;
        this.primaryController = primaryController;
        this.secondaryController = secondaryController;
        this.aprilTagHandler = aprilTagHandler;

        this.reverseIntake = ControlHandler.get(primaryController, ControllerConstants.INTAKE_REVERSE);
        this.runShooterHigh = ControlHandler.get(primaryController, ControllerConstants.SHOOTER_RUN_HIGH_SPEED);
        this.runShooterAmp = ControlHandler.get(primaryController, ControllerConstants.SHOOTER_RUN_AMP_SPEED);
        this.autoAimSubwoofer = ControlHandler.get(primaryController, ControllerConstants.AUTO_AIM_FOR_SHOOT);

        addRequirements(shooter, intake, conveyor);
    }

    @Override
    public void initialize() {
        ControlHandler.get(primaryController, ControllerConstants.INTAKE_RUN)
            .onTrue(toggleIntakeRun);
        
        ControlHandler.get(secondaryController, ControllerConstants.INTAKE_POS_TOGGLE)
            .onTrue(toggleIntakePos);

        ControlHandler.get(secondaryController, ControllerConstants.SHOOTER_IDLE_RPM_TOGGLE)
            .onTrue(toggleIdleShooterRpm);

        ControlHandler.get(secondaryController, ControllerConstants.TOGGLE_FLAP)
            .onTrue(toggleFlapStage);

        ControlHandler.get(secondaryController, ControllerConstants.WIGGLE_FLAP)
            .onTrue(wiggleFlap)
            .onFalse(wiggleFlap);
        ControlHandler.get(secondaryController, ControllerConstants.RESET_RING_HANDLING_STATE)
            .onTrue(resetState);
    }

    private double shooterAngle = 45;

    private double flapAngleTargetL = 155;
    private double flapAngleTargetR = 20;
    @Override
    public void execute() {
        if(this.flapStage == FlapStage.DOWN) {
            flapAngleTargetL = 155;
            flapAngleTargetR = 20;
        } else if(this.flapStage == FlapStage.MIDDLE) {
            flapAngleTargetL = 60;
            flapAngleTargetR = 115;
        } else if(this.flapStage == FlapStage.UP) {
            flapAngleTargetL = 45;
            flapAngleTargetR = 130;
        }

        this.shooter.flapToAngle(flapAngleTargetL, flapAngleTargetR);

        if(primaryController.getPOV() == 0) {
            shooterAngle++;
        } else if(primaryController.getPOV() == 180) {
            shooterAngle--;
        }

        Optional<Double> distanceToTargetOptional = getDistanceToTarget(Util.getTargetPose().toPose2d()); // reuse this optional
        hasAutoAimAprilTag.setBoolean(distanceToTargetOptional.isPresent()); // if the optional is present we've found a tag that we can lock to, so we can tell drivers this

        if(autoAimSubwoofer.getAsBoolean()) {
            autoAimShooterPivotAngle(distanceToTargetOptional).ifPresent(angle -> {
                shooterAngle = angle;
                System.out.println("pivoting shooter to " + angle + " to shoot at target");
            });
        }

        this.shooter.pivotToAngle(shooterAngle);

        if(intakeUp) {
            // our calibrate command sets our pivot target angle to ~80deg, but we don't want this command to override it early.
            // Instead, wait for the pivot's position to be changed (to the down position) before we bring it back up to the standard value.
            // This would ideally happen during autonomous, so when this command is actually scheduled, this state is never true. This is mainly in effect for in-pit calibration.
            if(this.intake.getPivotTargetAngle() < 77) {
                double throughboreValue = this.intake.getThroughboreEncoder().getAbsoluteEncoderValue();
                if (throughboreValue > 0 && throughboreValue < 3) {
                    this.intake.pivotToAngle(73 + throughboreValue);
                } else {
                    this.intake.pivotToAngle(73); // 73
                }
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
                // hasHandledNote = true;
                intakeUp = true;
            }
        }
    
        // System.out.println("note handled? " + hasHandledNote + ", has note: " + hasNote);

        if (reverseIntake.getAsBoolean()) { // eject takes priority
            shouldReverse = true;
        } else if (ColorSensor.isNoteColor()) {
            // System.out.println("NOTE FOUND");
            if(!hasHandledNote) {
                // shouldReverse = true;
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
            System.out.println("running shooter on high");
            if(!autoAimSubwoofer.getAsBoolean()) {
                System.out.println("not using subwoofer auto aim, going to high speed");
                shooter.setMotorRpm(2700);
                if (shooter.getShooterRpm() > 2700 - 100) {
                    this.conveyor.setShooterFeederMotorSpeed(1);
                    this.conveyor.setConveyorMotorsSpeed(-1);
                    hasHandledNote = false;
                    hasNote = false;
                }
            } else {
                System.out.println("attempting to use auto aim");
                distanceToTargetOptional
                    .map((distanceFromTarget) -> {
                        // Get the target RPM.
                        if(distanceFromTarget > 4.267) { // meters
                            return 2700.0;
                        } else {
                            return MathUtil.clamp(((distanceFromTarget + 1) / 4.267) * 2700 * 1.5, 1300, 2800);
                        }
                    }).ifPresentOrElse((targetRpm) -> {
                        System.out.println("using an auto-set target rpm of " + targetRpm);
                        shooter.setMotorRpm(targetRpm);

                        if (shooter.getShooterRpm() > targetRpm - 75) {
                            this.conveyor.setShooterFeederMotorSpeed(1);
                            this.conveyor.setConveyorMotorsSpeed(-1);
                            hasHandledNote = false;
                            hasNote = false;
                        }
                    }, () -> {
                        System.out.println("unable to use auto aim: no tag present.");
                    });
            }
        } else if(runShooterAmp.getAsBoolean()) {
            shooter.setMotorRpm(1400);
            if (shooter.getShooterRpm() > 1400 - 100) {
                this.conveyor.setShooterFeederMotorSpeed(1);
                this.conveyor.setConveyorMotorsSpeed(-1);
                hasHandledNote = false;
                hasNote = false;
            }
        } else {
            shooter.setMotorRpm(idleShooterRpm ? 1000 : 0); // 1300
        }
        // System.out.println("RPM: " + shooter.getShooterRpm());
        rpmPub.set(shooter.getShooterRpm());

        // this.intake.setPivotSpeed(-joystick.getRightVerticalMovement());

        // System.out.println("intake position: " + Util.nearestHundredth(intake.getPivotPosition()));

        // System.out.println("lower intake current: " + this.intake.getFollowerMotorCurrent() + ", top current: " + this.intake.getLeaderMotorCurrent());
        hasNotePub.setBoolean(hasNote);
        hasHandledNotePub.setBoolean(hasHandledNote);
    }

    private Optional<Double> autoAimShooterPivotAngle(Optional<Double> distanceToTargetOptional) {
        return distanceToTargetOptional.map((d) -> {
            double theta = Math.toRadians(1) + Math.atan((2 - 0.25) / (d - 0.17)); // trust me bro
            System.out.println("Distance to target: " + d);
            return Math.toDegrees(theta);
        });
    }

    private Optional<Double> getDistanceToTarget(Pose2d targetPose) {
        int tagId;
        if (Util.onBlueTeam()) {
            tagId = 7;
        } else {
            tagId = 4;
        }

        // this could be wrong IF
        // AprilTagHangle.RobotPoseAndTagDistance
        // tag distance is different than what can be derived just from
        // using the pose.
        // wait, why is this function taking a record that has the tag distance included in it,
        // discarding the tag distance, and then calculating the tag distance with another method?
        return this.aprilTagHandler
                .averageAutoAimPose(tagId)
                .map((robotPose) -> Math.sqrt(
                        Math.pow(targetPose.getY() - robotPose.getY(), 2) +
                        Math.pow(targetPose.getX() - robotPose.getX(), 2)
                ));
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
