package frc.robot.subsystems;

import edu.wpi.first.units.measure.MutVoltage;
import edu.wpi.first.units.measure.Voltage;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.robot.Constants.ShooterConstants;
import frc.robot.Constants.CANDevices;

import static edu.wpi.first.units.Units.Radians;
import static edu.wpi.first.units.Units.RadiansPerSecond;
import static edu.wpi.first.units.Units.Rotations;
import static edu.wpi.first.units.Units.RotationsPerSecond;
import static edu.wpi.first.units.Units.Volts;

import com.revrobotics.PersistMode;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.FeedbackSensor;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkFlexConfig;
import edu.wpi.first.units.measure.MutAngle;
import edu.wpi.first.units.measure.MutAngularVelocity;
import edu.wpi.first.units.measure.MutVoltage;


public class ShooterSys extends SubsystemBase {

    SparkFlex shooterMtr;
    RelativeEncoder shooterEnc;
    SparkClosedLoopController shooterController;

    public ShooterSys() {
        
        shooterMtr = new SparkFlex(CANDevices.shooterMtrId, MotorType.kBrushless);
        shooterEnc = shooterMtr.getEncoder();
        shooterController = shooterMtr.getClosedLoopController();

        SparkFlexConfig shooterConfig = new SparkFlexConfig();
        shooterConfig
            .inverted(true)
            .idleMode(IdleMode.kCoast)
            .smartCurrentLimit(ShooterConstants.stallLimitAmps, ShooterConstants.freeLimitAmps, ShooterConstants.maxRPM);
        shooterConfig.encoder
            .positionConversionFactor(1)
            .velocityConversionFactor(1);
        shooterConfig.closedLoop
            .feedbackSensor(FeedbackSensor.kPrimaryEncoder)
            .pid(ShooterConstants.shooterkP, ShooterConstants.shooterkI, ShooterConstants.shooterkD);

        shooterMtr.configure(shooterConfig, ResetMode.kResetSafeParameters, com.revrobotics.spark.SparkBase.PersistMode.kPersistParameters);

    }

    // Mutable holder for unit-safe voltage values, persisted to avoid reallocation.
    private final MutVoltage m_appliedVoltage = Volts.mutable(0);
    // Mutable holder for unit-safe linear distance values, persisted to avoid reallocation.
    private final MutAngle m_angle = Radians.mutable(0);
    // Mutable holder for unit-safe linear velocity values, persisted to avoid reallocation.
    private final MutAngularVelocity m_velocity = RadiansPerSecond.mutable(0);


    private final SysIdRoutine m_sysIdRoutine =
    new SysIdRoutine(
        // Empty config defaults to 1 volt/second ramp rate and 7 volt step voltage.
        new SysIdRoutine.Config(),
        new SysIdRoutine.Mechanism(
            // Tell SysId how to plumb the driving voltage to the motor(s).
            shooterMtr::setVoltage,
            // Tell SysId how to record a frame of data for each motor on the mechanism being
            // characterized.
            log -> {
                // Record a frame for the shooter motor.
                log.motor("shooter-wheel")
                    .voltage(
                        m_appliedVoltage.mut_replace(
                            shooterMtr.get() * RobotController.getBatteryVoltage(), Volts))
                    .angularPosition(m_angle.mut_replace(shooterEnc.getPosition(), Rotations))
                    .angularVelocity(
                        m_velocity.mut_replace(shooterEnc.getVelocity(), RotationsPerSecond));
              },
              // Tell SysId to make generated commands require this subsystem, suffix test state in
              // WPILog with this subsystem's name ("shooter")
              this));

    public void setShooterVoltage(Voltage volts) {
        shooterMtr.setVoltage(volts);
    }

    public void setShooterRPM(double rpm) {
        shooterController.setSetpoint(rpm, ControlType.kVelocity);
    }

    public double getShooterRPM() {
        return shooterEnc.getVelocity();
    }

    public void stop() {
        shooterController.setSetpoint(0, ControlType.kVelocity);
    }
    
    /**
   * Returns a command that will execute a quasistatic test in the given direction.
   *
   * @param direction The direction (forward or reverse) to run the test in
   */
  public Command sysIdQuasistatic(SysIdRoutine.Direction direction) {
    return m_sysIdRoutine.quasistatic(direction);
  }

  /**
   * Returns a command that will execute a dynamic test in the given direction.
   *
   * @param direction The direction (forward or reverse) to run the test in
   */
  public Command sysIdDynamic(SysIdRoutine.Direction direction) {
    return m_sysIdRoutine.dynamic(direction);
  }

}
