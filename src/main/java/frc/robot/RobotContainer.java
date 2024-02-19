package frc.robot;

import java.lang.reflect.Field;
import java.lang.reflect.Modifier;

import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandJoystick;
import edu.wpi.first.wpilibj2.command.button.CommandPS5Controller;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.commands.IntakeCommand;
import frc.robot.commands.ManualDriveCommand;
import frc.robot.commands.TestDriveCommand;
import frc.robot.commands.TestIntakeCommand;
import frc.robot.controllers.AbstractController;
import frc.robot.controllers.FlightJoystick;
import frc.robot.controllers.NintendoProController;
import frc.robot.controllers.PS5Controller;
import frc.robot.subsystems.swerve.DriveTrainSubsystem;
import frc.robot.subsystems.PowerHandler;
import frc.robot.subsystems.intake.IntakeSubsytem;
import frc.robot.subsystems.staticsubsystems.LimeLight;
import frc.robot.subsystems.staticsubsystems.RobotGyro;
import frc.robot.Constants.OperatorConstants;
import frc.robot.util.GyroPoseEstimator;
import frc.robot.util.NetworkTablesUtil;
import frc.robot.Constants.NetworkTablesConstants;

public class RobotContainer {
    private final DriveTrainSubsystem driveTrain;
    private final IntakeSubsytem intake;

    private final FlightJoystick driverController = new FlightJoystick(new CommandJoystick(OperatorConstants.RIGHT_JOYSTICK_PORT));
    private final NintendoProController nintendoProController = new NintendoProController(new CommandXboxController(OperatorConstants.NINTENDO_PRO_CONTROLLER));
    private final PS5Controller ps5Controller = new PS5Controller(new CommandPS5Controller(OperatorConstants.PS5_CONTROLLER));

    private final PowerHandler powerHandler = new PowerHandler();

    public final AbstractController primaryController = Flags.Operator.USING_NINTENDO_SWITCH_CONTROLLER ? this.nintendoProController : this.ps5Controller;

    private final GyroPoseEstimator gyroPoseEstimator = new GyroPoseEstimator();
    public RobotContainer() {
        if(Flags.DriveTrain.IS_ATTACHED) {
            this.driveTrain = new DriveTrainSubsystem();
        } else {
            this.driveTrain = null;
        }

        if(Flags.Intake.IS_ATTACHED) {
            this.intake = new IntakeSubsytem();
        } else {
            this.intake = null;
        }

        configureBindings();

        // Initialize static subsystems (this is a Java thing don't worry about it just copy it so that static blocks run on startup)
        LimeLight.poke();
        RobotGyro.poke();
    }

    private void configureBindings() {
        if(Flags.DriveTrain.IS_ATTACHED) {
            this.primaryController.upperButton().onTrue(this.driveTrain.rotateToAbsoluteZeroCommand());
        }
        this.primaryController.leftButton().onTrue(Commands.runOnce(() -> RobotGyro.resetGyroAngle()));
    }

    public void onRobotInit() {
        printFlagsClass();
    }

    private static void printFlagsClass() {
        try {
            Class<Flags> clazz = Flags.class;
            var flagsTable = NetworkTablesUtil.MAIN_ROBOT_TABLE.getSubTable(NetworkTablesConstants.MAIN_TABLE_NAME).getSubTable("Flags");

            for(Field field : clazz.getDeclaredFields()) {
                if(Modifier.isStatic(field.getModifiers())) {
                    try {
                        flagsTable.getEntry(field.getName()).setValue(field.get(null));
                    } catch(IllegalAccessException e) {
                        System.out.println("Unable to upload value of Flags field " + field.getName());
                    }
                }
            }

            for(Class<?> c : clazz.getClasses()) {
                var subTable = flagsTable.getSubTable(c.getSimpleName());
                for(Field field : c.getDeclaredFields()) {
                    if(Modifier.isStatic(field.getModifiers())) {
                        try {
                            subTable.getEntry(field.getName()).setValue(field.get(null));
                        } catch(IllegalAccessException e) {
                            System.out.println("Unable to upload value from Flags subclass " + c.getName() + ", field " + field.getName());
                        }
                    }
                }
            }
        } catch(Exception e) {
            System.out.println("error while uploading the flags classes:");
            e.printStackTrace();
        }
    }

    public void onAutonInit() {

    }

    public void onTeleopInit() {
        if(Flags.DriveTrain.IS_ATTACHED) {
            if(Flags.DriveTrain.USE_TEST_DRIVE_COMMAND) {
                this.driveTrain.setDefaultCommand(new TestDriveCommand(this.driveTrain, this.primaryController));
            } else {
                this.driveTrain.setDefaultCommand(new ManualDriveCommand(this.driveTrain, this.primaryController));
            }
        }

        if(Flags.Intake.IS_ATTACHED) {
            if(Flags.Intake.USE_TEST_INTAKE_COMMAND) {
                this.intake.setDefaultCommand(new TestIntakeCommand(this.intake, this.primaryController));
            } else {
                this.intake.setDefaultCommand(new IntakeCommand(this.intake, this.primaryController));
            }
        }
    }

    public void onTeleopPeriodic() {
        this.powerHandler.updateNT();
    }

    public void onRobotPeriodic() {
        this.gyroPoseEstimator.update();
    }
}
