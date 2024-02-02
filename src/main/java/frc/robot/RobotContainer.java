package frc.robot;

import java.lang.reflect.Field;
import java.lang.reflect.Modifier;

import edu.wpi.first.networktables.BooleanPublisher;
import edu.wpi.first.wpilibj2.command.button.CommandJoystick;
import edu.wpi.first.wpilibj2.command.button.CommandPS5Controller;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.commands.ManualDrive;
import frc.robot.controllers.AbstractController;
import frc.robot.controllers.FlightJoystick;
import frc.robot.controllers.NintendoProController;
import frc.robot.controllers.PS5Controller;
import frc.robot.subsystems.swerve.DriveTrainSubsystem;
import frc.robot.subsystems.staticsubsystems.LimeLight;
import frc.robot.subsystems.staticsubsystems.RobotGyro;
import frc.robot.Constants.OperatorConstants;
import frc.robot.util.NetworkTablesUtil;
import frc.robot.Constants.NetworkTablesConstants;

public class RobotContainer {
    private final DriveTrainSubsystem driveTrain = new DriveTrainSubsystem();

    public final FlightJoystick driverController = new FlightJoystick(new CommandJoystick(OperatorConstants.RIGHT_JOYSTICK_PORT));
    public final NintendoProController nintendoProController = new NintendoProController(new CommandXboxController(OperatorConstants.NINTENDO_PRO_CONTROLLER));
    public final PS5Controller ps5Controller = new PS5Controller(new CommandPS5Controller(OperatorConstants.PS5_CONTROLLER));

    public final AbstractController primaryController = this.ps5Controller;

    public RobotContainer() {
        configureBindings();

        // Initialize static subsystems (this is a Java thing don't worry about it just copy it so that static blocks run on startup)
        LimeLight.poke();
        RobotGyro.poke();
    }

    private void configureBindings() {
        /*
        this.nintendoProController.controller.button(4).whileTrue(new InstantCommand(() -> {
            System.out.println("GO ZERO");
            this.driveTrain.rotateModulesToAbsoluteZero();
            System.out.println("DONE");
        }, this.driveTrain));*/
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
                var subTable = flagsTable.getSubTable(c.getCanonicalName());
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
            System.out.println("error while uploading the flags classes");
        }
    }

    public void onAutonInit() {

    }

    public void onTeleopInit() {
        this.driveTrain.setDefaultCommand(new ManualDrive(this.driveTrain, this.primaryController));
    }

    BooleanPublisher a = NetworkTablesUtil.MAIN_ROBOT_TABLE.getSubTable("isaac_man").getBooleanTopic("a button press").publish();

    public void onTeleopPeriodic() {
        a.set(this.primaryController.getRawButtonWrapper(2));
    }
}
