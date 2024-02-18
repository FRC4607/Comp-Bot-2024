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
 * characterizing the kS value of a drive motor on a swerve drive.
 */
public class SlipCurrentTest implements SwerveRequest {
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
            request.FeedForward = SmartDashboard.getNumber("Slip Test Current", 0);
            module.apply(ZERO_STATE, DriveRequestType.Velocity);
        }
        return StatusCode.OK;
    }
}