package frc.robot.subsystems.swerve.real;

import frc.robot.utils.OptixSpark;
import frc.robot.utils.UtilityFunctions;

import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.hardware.CANcoder;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.units.measure.Angle;
import frc.robot.subsystems.swerve.SwerveModuleIO;
import frc.robot.subsystems.swerve.SwerveConstants.DrivetrainConstants;
import frc.robot.subsystems.swerve.SwerveConstants.MotorConstants;
import frc.robot.utils.LoggedTunableNumber;
import frc.robot.utils.MiscConstants.MotorControllerConstants;

public class SwerveModuleSpark implements SwerveModuleIO {
    private OptixSpark drive;
    private OptixSpark turn;

    private CANcoder absoluteEncoder;

    private double absoluteEncoderOffsetRad;

    public LoggedTunableNumber module1Offset = new LoggedTunableNumber("Swerve/Module1Offset", 0);
    public LoggedTunableNumber module4Offset = new LoggedTunableNumber("Swerve/Module2Offset", 0);

    int idx = 0;

    public SwerveModuleSpark(int index) {
        drive = new OptixSpark(MotorConstants.driveMotorIds[index], OptixSpark.Type.SPARKMAX);

        drive.setPositionConversionFactor(
                1 / MotorConstants.driveMotorGearRatio * Math.PI * DrivetrainConstants.wheelDiameterMeters);
        drive.setVelocityConversionFactor(
                1 / MotorConstants.driveMotorGearRatio * Math.PI * DrivetrainConstants.wheelDiameterMeters
                        * (1 / 60.0));
        drive.setInverted(MotorConstants.driveMotorInverted[index]);
        drive.setCurrentLimit(MotorControllerConstants.standardStallLimit,
                MotorControllerConstants.standardFreeLimit);
        drive.setBrakeMode(true);

        turn = new OptixSpark(MotorConstants.turnMotorIds[index], OptixSpark.Type.SPARKMAX);
        turn.setPositionConversionFactor(1 / MotorConstants.turnMotorGearRatio * Math.PI * 2);
        turn.setVelocityConversionFactor(1 / MotorConstants.turnMotorGearRatio * Math.PI * 2 * (1 / 60.0));
        turn.setInverted(MotorConstants.turningMotorInverted[index]);
        turn.setCurrentLimit(MotorControllerConstants.standardStallLimit,
                MotorControllerConstants.standardFreeLimit);
        turn.setBrakeMode(true);
        turn.setPositionWrapping(0, 2.0 * Math.PI);

        drive.applyConfig();
        turn.applyConfig();

        absoluteEncoder = new CANcoder(MotorConstants.absoluteEncoderIds[index]);
        absoluteEncoderOffsetRad = MotorConstants.absoluteEncoderOffsetRad[index];
        turn.setPosition(
                absoluteEncoder.getPosition().getValueAsDouble() * 2.0 * Math.PI - absoluteEncoderOffsetRad);

        absoluteEncoder.optimizeBusUtilization();
        StatusSignal<Angle> absolutePositionSignal = absoluteEncoder.getAbsolutePosition();
        absolutePositionSignal.setUpdateFrequency(100);

        idx = index;
    }

    @Override
    public void updateData(ModuleData data) {
        data.driveAppliedVolts = drive.getAppliedVolts();
        data.drivePositionM = drive.getPosition();
        data.driveVelocityMPerSec = drive.getVelocity();
        data.driveCurrentAmps = drive.getCurrent();
        data.driveTempCelcius = drive.getTemperature();

        data.turnAppliedVolts = turn.getAppliedVolts();
        data.turnPositionRad = turn.getPosition();

        data.absoluteEncoderPositionRad = absoluteEncoder.getPosition().getValueAsDouble() * 2 * Math.PI
                - absoluteEncoderOffsetRad;
        data.turnVelocityRadPerSec = turn.getVelocity();
        data.turnCurrentAmps = turn.getCurrent();
        data.turnTempCelcius = turn.getTemperature();

        // data.turnPositionRad = data.absoluteEncoderPositionRad;
    };

    @Override
    public void setDriveVelocity(double setpointVelocity, double feedforward) {
        drive.setVelocityControl(setpointVelocity, feedforward);
    }

    @Override
    public void setTurnPosition(double setpointVelocity, double feedforward) {
        turn.setVelocityControl(setpointVelocity, feedforward);
    }

    /** Run the drive motor at the specified voltage. */
    @Override
    public void setDriveVoltage(double volts) {
        volts = MathUtil.clamp(volts, -12, 12);
        volts = UtilityFunctions.applyDeadband(volts, MotorControllerConstants.deadbandVoltage);

        // volts = Math.copySign(12, volts);
        drive.setVoltage(volts);
    }

    /** Run the turn motor at the specified voltage. */
    @Override
    public void setTurnVoltage(double volts) {
        volts = MathUtil.clamp(volts, -12, 12);
        volts = UtilityFunctions.applyDeadband(volts, MotorControllerConstants.deadbandVoltage);
        turn.setVoltage(volts);
    }

    /** Enable or disable brake mode on the drive motor. */
    @Override
    public void setDriveBrakeMode(boolean enable) {
        // drive.setBrakeMode(enable);
    }

    /** Enable or disable brake mode on the turn motor. */
    @Override
    public void setTurningBrakeMode(boolean enable) {
        // turn.setBrakeMode(enable);
    }

    @Override
    public void syncEncoderPosition() {
        turn.setPosition(absoluteEncoder.getPosition().getValueAsDouble() * 2.0 * Math.PI - absoluteEncoderOffsetRad);
    }
}
