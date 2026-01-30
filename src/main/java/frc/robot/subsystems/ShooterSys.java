package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ShooterConstants;
import frc.robot.Constants.CANDevices;

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


public class ShooterSys extends SubsystemBase {

    private final SparkFlex shooterMtr;
    private final RelativeEncoder shooterEnc;
    private final SparkClosedLoopController shooterController;

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

    public void setShooterRPM(double rpm) {
        shooterController.setReference(rpm, ControlType.kVelocity);
    }

    public double getShooterRPM() {
        return shooterEnc.getVelocity();
    }

    public void stop() {
        shooterController.setReference(0, ControlType.kVelocity);
    }
    
}
