package frc.robot;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.CommandJoystick;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.commands.ManualDrive;
import frc.robot.controllers.FlightJoystick;
import frc.robot.controllers.NintendoProController;
import frc.robot.subsystems.swerve.DriveTrainSubsystem;
import frc.robot.subsystems.staticsubsystems.LimeLight;
import frc.robot.subsystems.staticsubsystems.RobotGyro;
import frc.robot.Constants.OperatorConstants;

public class RobotContainer {
    private final DriveTrainSubsystem driveTrain = new DriveTrainSubsystem();

    public final FlightJoystick driverController = new FlightJoystick(new CommandJoystick(OperatorConstants.RIGHT_JOYSTICK_PORT));
    public final NintendoProController nintendoProController = new NintendoProController(new CommandXboxController(OperatorConstants.NINTENDO_PRO_CONTROLLER));

    public RobotContainer() {
        configureBindings();

        // Initialize static subsystems (this is a Java thing don't worry about it just copy it so that static blocks run on startup)
        LimeLight.poke();
        RobotGyro.poke();
    }

    private void configureBindings() {
        /*
        this.nintendoProController.controller.button(4).whileTrue(new InstantCommand(() -> {
            System.out.println("GO ZERO UWU");
            this.driveTrain.rotateModulesToAbsoluteZero();
            System.out.println("DONE");
        }, this.driveTrain));*/
    }

    public void onRobotInit() {

    }

    public void onAutonInit() {

    }

    public void onTeleopInit() {
        this.driveTrain.setDefaultCommand(new ManualDrive(this.driveTrain, this.nintendoProController));
    }
}
