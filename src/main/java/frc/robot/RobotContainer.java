package frc.robot;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;

import edu.wpi.first.networktables.GenericPublisher;
import edu.wpi.first.networktables.NetworkTableType;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.CommandJoystick;
import edu.wpi.first.wpilibj2.command.button.CommandPS5Controller;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.Constants.OperatorConstants;
import frc.robot.Constants.OperatorConstants.ControllerConstants;
import frc.robot.commands.*;
import frc.robot.commands.auto.RingHandlingAutonCommand;
import frc.robot.commands.auto.RunShooterAutonCommand;
import frc.robot.controllers.AbstractController;
import frc.robot.controllers.FlightJoystick;
import frc.robot.controllers.NintendoProController;
import frc.robot.controllers.PS5Controller;
import frc.robot.subsystems.PowerHandler;
import frc.robot.subsystems.climber.ClimberSubsystem;
import frc.robot.subsystems.conveyor.ConveyorSubsystem;
import frc.robot.subsystems.intake.IntakeSubsystem;
import frc.robot.subsystems.shooter.ShooterSubsystem;
import frc.robot.subsystems.staticsubsystems.ColorSensor;
import frc.robot.subsystems.staticsubsystems.LimeLight;
import frc.robot.subsystems.staticsubsystems.RobotGyro;
import frc.robot.subsystems.swerve.DriveTrainSubsystem;
import frc.robot.util.AprilTagHandler;
import frc.robot.util.ControlHandler;
import frc.robot.util.GyroPoseEstimator;
import frc.robot.util.NetworkTablesUtil;
import frc.robot.util.Util;

import java.lang.reflect.Field;
import java.lang.reflect.Modifier;

public class RobotContainer {
    private static final GenericPublisher COLOR_SENSOR_PUB = NetworkTablesUtil.getPublisher("robot", "color_sensor_sees_note", NetworkTableType.kBoolean);

    private final DriveTrainSubsystem driveTrain;
    private final IntakeSubsystem intake;
    private final ShooterSubsystem shooter;
    private final ConveyorSubsystem conveyor;
    private final ClimberSubsystem climber;

    private final FlightJoystick sideJoystick = new FlightJoystick(new CommandJoystick(OperatorConstants.RIGHT_JOYSTICK_PORT));
    private final NintendoProController nintendoProController = new NintendoProController(new CommandXboxController(OperatorConstants.NINTENDO_PRO_CONTROLLER));
    private final PS5Controller ps5Controller = new PS5Controller(new CommandPS5Controller(OperatorConstants.PS5_CONTROLLER));
    private final AbstractController primaryController = Flags.Operator.NINTENDO_SWITCH_CONTROLLER_AS_PRIMARY ? this.nintendoProController : this.ps5Controller;
    private final PowerHandler powerHandler = new PowerHandler();
    private final GyroPoseEstimator gyroPoseEstimator = new GyroPoseEstimator();
    private final AprilTagHandler aprilTagHandler = new AprilTagHandler();

    private final SendableChooser<Command> autonChooser;

    public RobotContainer() {
        this.driveTrain = Util.createIfFlagElseNull(() -> new DriveTrainSubsystem(aprilTagHandler), Flags.DriveTrain.IS_ATTACHED);
        this.intake     = Util.createIfFlagElseNull(IntakeSubsystem::new, Flags.Intake.IS_ATTACHED);
        this.shooter    = Util.createIfFlagElseNull(ShooterSubsystem::new, Flags.Shooter.IS_ATTACHED);
        this.conveyor   = Util.createIfFlagElseNull(ConveyorSubsystem::new, Flags.Conveyor.IS_ATTACHED);
        this.climber    = Util.createIfFlagElseNull(ClimberSubsystem::new, Flags.Climber.IS_ATTACHED);

        NamedCommands.registerCommand("shoot", new RunShooterAutonCommand(this.shooter, this.conveyor, 1900, 54));
        NamedCommands.registerCommand("shoot_lower", new RunShooterAutonCommand(this.shooter, this.conveyor, 2600, 45));
        NamedCommands.registerCommand("shoot_40", new RunShooterAutonCommand(shooter, conveyor, 2600, 40));
        NamedCommands.registerCommand("drop_intake", new InstantCommand(() -> this.intake.pivotToAngle(0), this.intake));
        NamedCommands.registerCommand("activate_intake", new RingHandlingAutonCommand(this.conveyor, this.intake));
        NamedCommands.registerCommand("raise_intake", new InstantCommand(() -> this.intake.pivotToAngle(74), this.intake));
        NamedCommands.registerCommand("adjust", new InstantCommand());
        NamedCommands.registerCommand("shoot_30", new RunShooterAutonCommand(shooter, conveyor, 2600, 30));

        configureBindings();

        // Initialize static subsystems (this is a Java thing don't worry about it just copy it so that static blocks run on startup)
        LimeLight.poke();
        RobotGyro.poke();
        ColorSensor.initialize();

        if(Flags.DriveTrain.IS_ATTACHED) {
            this.autonChooser = AutoBuilder.buildAutoChooser();
            SmartDashboard.putData("choose your auto", this.autonChooser);
        } else {
            this.autonChooser = null;
        }

        NetworkTablesUtil.getConnections();
    }

    /**
     * This method is used to upload the contents of the {@link frc.robot.Flags Flags} class to NetworkTables.
     * This is useful for debugging since the flags control robot functionality and allow an uploaded method to quickly check which parts of the robot code are enabled or disabled.
     */
    private static void uploadFlagsClass() {
        try {
            Class<Flags> clazz = Flags.class;
            var flagsTable = NetworkTablesUtil.MAIN_ROBOT_TABLE.getSubTable("Flags");

            for (Field field : clazz.getDeclaredFields()) {
                if (Modifier.isStatic(field.getModifiers())) {
                    try {
                        flagsTable.getEntry(field.getName()).setValue(field.get(null));
                    } catch (IllegalAccessException e) {
                        System.out.println("Unable to upload value of Flags field " + field.getName());
                    }
                }
            }

            for (Class<?> c : clazz.getClasses()) {
                var subTable = flagsTable.getSubTable(c.getSimpleName());
                for (Field field : c.getDeclaredFields()) {
                    if (Modifier.isStatic(field.getModifiers())) {
                        try {
                            subTable.getEntry(field.getName()).setValue(field.get(null));
                        } catch (IllegalAccessException e) {
                            System.out.println("Unable to upload value from Flags subclass " + c.getName() + ", field " + field.getName());
                        }
                    }
                }
            }
        } catch (Exception e) {
            System.out.println("error while uploading the flags classes:");
            e.printStackTrace();
        }
    }

    private void configureBindings() {
        if (Flags.DriveTrain.IS_ATTACHED) {
            ControlHandler.get(this.nintendoProController, ControllerConstants.ZERO_SWERVE_MODULES).onTrue(this.driveTrain.rotateToAbsoluteZeroCommand());
        }
        ControlHandler.get(this.nintendoProController, ControllerConstants.ZERO_GYRO).onTrue(Commands.runOnce(() -> {
            if(Util.onBlueTeam()) {
                RobotGyro.resetGyroAngle();
            } else {
                RobotGyro.setGyroAngle(180);
            }
            this.driveTrain.setHeadingLockMode(false);
        }));

        sideJoystick.joystick.button(8).whileTrue(new CalibrateIntakeCommand(intake, nintendoProController));
        if(Flags.DriveTrain.IS_ATTACHED) {
            ControlHandler.get(this.nintendoProController, ControllerConstants.RESET_POSE_ESTIMATOR).onTrue(new InstantCommand(() -> this.driveTrain.resetPoseToMidSubwoofer()));
        }
    }

    public void onRobotInit() {
        uploadFlagsClass();
    }

    public void onTeleopInit() {
        this.getAutonomousCommand().cancel();

        if (Flags.DriveTrain.IS_ATTACHED) {
            RobotGyro.setGyroAngle(this.driveTrain.getPose().getRotation().getDegrees());
            if (Flags.DriveTrain.USE_TEST_DRIVE_COMMAND) {
                this.driveTrain.setDefaultCommand(new TestDriveCommand(this.driveTrain, this.primaryController));
            } else {
                this.driveTrain.setDefaultCommand(new ManualDriveCommand(this.driveTrain, this.primaryController));
            }
        }

        if (Flags.Intake.IS_ATTACHED) {
            if (Flags.Intake.USE_TEST_INTAKE_COMMAND) {
                this.intake.setDefaultCommand(new TestIntakeCommand(this.intake, this.primaryController));
            } else if (Flags.Conveyor.IS_ATTACHED && Flags.Shooter.IS_ATTACHED) {
                this.intake.setDefaultCommand(new RingHandlingCommand(shooter, intake, conveyor, this.primaryController, this.nintendoProController, this.driveTrain::getPose));
            } else {
                this.intake.setDefaultCommand(new IntakeCommand(this.intake, this.primaryController));
            }
        }

        if (Flags.Shooter.IS_ATTACHED) {
            if (Flags.Shooter.USE_TEST_SHOOTER_COMMAND) {
                this.shooter.setDefaultCommand(new TestShooterCommand(this.shooter, this.primaryController));
            } else {

            }
        }

        if(Flags.Climber.IS_ATTACHED) {
            if(Flags.Climber.USE_TEST_CLIMBER_COMMAND && !Flags.Operator.NINTENDO_SWITCH_CONTROLLER_AS_PRIMARY) {
                this.climber.setDefaultCommand(new TestClimberCommand(climber, this.nintendoProController));
            }
        }

        /*
        if(Flags.Conveyor.IS_ATTACHED) {
            if(Flags.Conveyor.USE_TEST_CONVEYOR_COMMAND) {
                this.conveyor.setDefaultCommand(new TestConveyorCommand(this.conveyor, this.primaryController));
            } else {
                
            }
        }*/
    }

    public Command getAutonomousCommand() {
        return this.autonChooser.getSelected();
    }

    public void onTeleopPeriodic() {
        this.powerHandler.updateNT();
    }
    
    public void onRobotPeriodic() {
        COLOR_SENSOR_PUB.setBoolean(ColorSensor.isNoteColor());
    }
}
