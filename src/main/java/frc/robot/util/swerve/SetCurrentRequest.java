package frc.robot.util.swerve;

import java.lang.reflect.Field;

import com.ctre.phoenix6.StatusCode;
import com.ctre.phoenix6.controls.VelocityTorqueCurrentFOC;
import com.ctre.phoenix6.mechanisms.swerve.SwerveModule;
import com.ctre.phoenix6.mechanisms.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.mechanisms.swerve.SwerveRequest;

import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

/**
 * A {@link com.ctre.phoenix6.mechanisms.swerve.SwerveRequest} for
 * applying a given current value to all of the wheels of a swerve drive. This
 * works by hijacking the feedforward parameter of the velocity control request,
 * so ensure the PIDFF constants of the drive wheels are set to 0 in
 * {@link frc.robot.Calibrations.DrivetrainCalibrations}. If you expect to
 * command current values over the slip current limit, make sure to set that
 * value to 800.0 as well.
 */
public class SetCurrentRequest implements SwerveRequest {
    private static final SwerveModuleState ZERO_STATE = new SwerveModuleState();

    @Override
    public StatusCode apply(SwerveControlRequestParameters parameters, SwerveModule... modules) {
        for (SwerveModule module : modules) {
            VelocityTorqueCurrentFOC request;
            try {
                Field requestField = module.getClass().getDeclaredField("m_velocityTorqueSetter");
                requestField.setAccessible(true);
                request = (VelocityTorqueCurrentFOC) requestField.get(module);
            } catch (NoSuchFieldException e) {
                // uh oh
                System.err.println(e.toString());
                return StatusCode.GeneralError;
            } catch (IllegalAccessException e) {
                // uh oh
                System.err.println(e.toString());
                return StatusCode.GeneralError;
            }
            request.FeedForward = SmartDashboard.getNumber("Set Current Request", 0);
            module.apply(ZERO_STATE, DriveRequestType.Velocity);
        }
        return StatusCode.OK;
    }
}