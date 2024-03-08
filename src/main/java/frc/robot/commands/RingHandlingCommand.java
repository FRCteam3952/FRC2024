package frc.robot.commands;

import edu.wpi.first.networktables.DoublePublisher;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.OperatorConstants.ControllerConstants;
import frc.robot.subsystems.conveyor.ConveyorSubsystem;
import frc.robot.subsystems.intake.IntakeSubsytem;
import frc.robot.subsystems.shooter.ShooterSubsystem;
import frc.robot.util.ControlHandler;
import frc.robot.util.NetworkTablesUtil;
import frc.robot.controllers.AbstractController;

/**
 * also known as SonicTheHedgehogCommand
 */
public class RingHandlingCommand extends Command {
    private final ShooterSubsystem shooter;
    private final IntakeSubsytem intake;
    private final ConveyorSubsystem conveyor;
    private final AbstractController joystick;
    
    private static final DoublePublisher rpmPub = NetworkTablesUtil.MAIN_ROBOT_TABLE.getDoubleTopic("shooter_rpm").publish();
    public RingHandlingCommand(ShooterSubsystem shooter, IntakeSubsytem intake, ConveyorSubsystem conveyor, AbstractController joystick) {
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
        if (ControlHandler.get(joystick, ControllerConstants.INTAKE_RUN).getAsBoolean()) {
            this.intake.setIntakeSpeed(0.8, 0.8);
            this.conveyor.setConveyorMotorsSpeed(1);
            this.conveyor.setShooterFeederMotorSpeed(1);
        } else if (ControlHandler.get(joystick, ControllerConstants.INTAKE_REVERSE).getAsBoolean()) {
            this.intake.setIntakeSpeed(-0.2, -0.2);
            this.conveyor.setConveyorMotorsSpeed(-0.2);
            this.conveyor.setShooterFeederMotorSpeed(-0.2);
        } else {
            this.intake.setIntakeSpeed(0, 0);
            this.conveyor.setConveyorMotorsSpeed(0);
            this.conveyor.setShooterFeederMotorSpeed(0);
        }

        // when the shooter is up high enough we GO BRRRR
        if(joystick.rightShoulderTrigger().getAsBoolean()) {
            shooter.setMotorRpm(700);
            if(shooter.getShooterRpm() > 500) {
                this.conveyor.setShooterFeederMotorSpeed(1);
            }
        } else {
            shooter.stopShooterPID();
        }
        // System.out.println("RPM: " + shooter.getShooterRpm());
        // rpmPub.set(shooter.getShooterRpm());

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
