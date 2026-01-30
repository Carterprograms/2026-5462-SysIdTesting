package frc.robot;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;
import com.pathplanner.lib.events.EventTrigger;

import edu.wpi.first.cameraserver.CameraServer;
import edu.wpi.first.cscore.UsbCamera;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import frc.robot.Constants.ButtonPanelConstants;
import frc.robot.Constants.ControllerConstants;
import frc.robot.Constants.DriveConstants;
import frc.robot.commands.drivetrain.ArcadeDriveCmd;
import frc.robot.commands.drivetrain.LockCmd;
import frc.robot.subsystems.SwerveSys;
import frc.robot.subsystems.SwerveRotation;
import frc.robot.commands.drivetrain.PointCmd;
import frc.robot.commands.drivetrain.AutoShootCmd;
import frc.robot.subsystems.ShooterSys;
import frc.robot.commands.drivetrain.AutoAimCmd;
import frc.robot.subsystems.IntakeSys;
import frc.robot.commands.functions.IntakeCmd;

public class RobotContainer {
    
    // Initialize subsystems.
    private final SwerveSys swerveSys = new SwerveSys();
    private final SwerveRotation swerveRotation = new SwerveRotation(swerveSys);
    private final ShooterSys shooterSys = new ShooterSys();
    private final IntakeSys intakeSys = new IntakeSys();

    //Initialize joysticks.
    public final static CommandXboxController driverController = new CommandXboxController(ControllerConstants.driverGamepadPort);
    public final static CommandXboxController operatorController = new CommandXboxController(ControllerConstants.operatorGamepadPort);
    public final static Joystick ButtonPanel = new Joystick(ControllerConstants.operatorGamepadPort);
    

    //Name Commands
    private final PointCmd pointCmd;
    private final AutoShootCmd testCmd;
    private final AutoAimCmd autoPointCmd;
    private final IntakeCmd intakeCmd;

    //Initialize auto selector.
    SendableChooser<Command> autoSelector = new SendableChooser<Command>();

    private UsbCamera camera;

    public RobotContainer() {
        RobotController.setBrownoutVoltage(DriveConstants.brownoutVoltage);

        camera = new UsbCamera("driver camera", 2);
        
        CameraServer.startAutomaticCapture(camera);

        // Register Commands to PathPlanner
        NamedCommands.registerCommand("Aim", new AutoAimCmd(swerveSys));
        NamedCommands.registerCommand("Shoot", new AutoShootCmd(shooterSys));
        NamedCommands.registerCommand("Intake", new IntakeCmd(intakeSys));

        // Build an auto chooser. This will use Commands.none() as the default option.
        autoSelector = AutoBuilder.buildAutoChooser();

        SmartDashboard.putData("auto select", autoSelector);

    //Initalize Commands
        pointCmd = new PointCmd(swerveRotation);
        testCmd = new AutoShootCmd(shooterSys);
        autoPointCmd = new AutoAimCmd(swerveSys);
        intakeCmd = new IntakeCmd(intakeSys);

        //Add Requirements
    // pointCmd already requires the lightweight rotation subsystem. No need to add SwerveSys requirement.
            

        //new EventTrigger("Aim").whileTrue(new PointCmd(swerveSys));


        configDriverBindings();
        configButtonPanel();

    }

    private void configButtonPanel() {
    }

    public void configDriverBindings() {
        swerveSys.setDefaultCommand(new ArcadeDriveCmd(
            () -> MathUtil.applyDeadband(driverController.getLeftY(), ControllerConstants.joystickDeadband),
            () -> MathUtil.applyDeadband(driverController.getLeftX(), ControllerConstants.joystickDeadband),
            () -> MathUtil.applyDeadband(driverController.getRightX(), ControllerConstants.joystickDeadband),
            true,
            true,
            swerveSys));

        driverController.start().onTrue(Commands.runOnce(() -> swerveSys.resetHeading()));

        //Swerve locking system
        driverController.axisGreaterThan(XboxController.Axis.kLeftTrigger.value, ControllerConstants.triggerPressedThreshhold)
           .whileTrue(new LockCmd(swerveSys));

    driverController.rightBumper().whileTrue(new PointCmd(swerveRotation));
    driverController.a().whileTrue(new IntakeCmd(intakeSys));
    driverController.b().whileTrue(new AutoShootCmd(shooterSys));
    }

    public Command getAutonomousCommand() {
        return autoSelector.getSelected();
    } 

    // For uniformity, any information sent to Shuffleboard/SmartDashboard should go here.
    public void updateInterface() {
        
        SmartDashboard.putNumber("heading degrees", swerveSys.getHeading().getDegrees());
        SmartDashboard.putNumber("speed m/s", swerveSys.getAverageDriveVelocityMetersPerSec());

        SmartDashboard.putNumber("pose x meters", swerveSys.getPose().getX());
        SmartDashboard.putNumber("pose y meters", swerveSys.getPose().getY());

        SmartDashboard.putNumber("blue pose x meters", swerveSys.getBlueSidePose().getX());

        SmartDashboard.putNumber("FL angle degrees", swerveSys.getModuleStates()[0].angle.getDegrees());
        SmartDashboard.putNumber("FR angle degrees", swerveSys.getModuleStates()[1].angle.getDegrees());
        SmartDashboard.putNumber("BL angle degrees", swerveSys.getModuleStates()[2].angle.getDegrees());
        SmartDashboard.putNumber("BR angle degrees", swerveSys.getModuleStates()[3].angle.getDegrees());

        SmartDashboard.putNumber("FL raw CANCoder degrees", swerveSys.getCanCoderAngles()[0].getDegrees());
        SmartDashboard.putNumber("FR raw CANCoder degrees", swerveSys.getCanCoderAngles()[1].getDegrees());
        SmartDashboard.putNumber("BL raw CANCoder degrees", swerveSys.getCanCoderAngles()[2].getDegrees());
        SmartDashboard.putNumber("BR raw CANCoder degrees", swerveSys.getCanCoderAngles()[3].getDegrees());

        SmartDashboard.putNumber("FL offset CANCoder degrees", swerveSys.getCanCoderAngles()[0].getDegrees() - DriveConstants.frontLeftModOffset.getDegrees());
        SmartDashboard.putNumber("FR offset CANCoder degrees", swerveSys.getCanCoderAngles()[1].getDegrees() - DriveConstants.frontRightModOffset.getDegrees());
        SmartDashboard.putNumber("BL offset CANCoder degrees", swerveSys.getCanCoderAngles()[2].getDegrees() - DriveConstants.backLeftModOffset.getDegrees());
        SmartDashboard.putNumber("BR offset CANCoder degrees", swerveSys.getCanCoderAngles()[3].getDegrees() - DriveConstants.backRightModOffset.getDegrees());

        SmartDashboard.putNumber("drive voltage", swerveSys.getAverageDriveVoltage());

    }   
}
