package org.firstinspires.ftc.teamcode.api;

import android.text.method.Touch;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.TouchSensor;

import java.util.List;

public class LimitedMotorX extends DcMotorX {

    // Touch sensor limits
    private TouchSensor limitLower = null;
    private TouchSensor limitUpper = null;

    // Encoded limits
    private Double positionLimitLower = null;
    private Double positionLimitUpper = null;

    // LimitedMotorX uses the same exact constructors as DcMotorX

    public LimitedMotorX(DcMotor motor){
        super(motor, 0, 0);
    }

    public LimitedMotorX(DcMotor motor, int ticksPerRev, double distancePerRev){
        super(motor, ticksPerRev, distancePerRev);
    }

    // Set a lower TouchSensor limit
    public void setLowerLimit(TouchSensor limitLower){
        this.limitLower = limitLower;
    }

    // Set a lower encoder limit
    public void setLowerLimit(double positionLimitLower) {
        this.positionLimitLower = positionLimitLower;
    }

    // Set an upper TouchSensor limit
    public void setUpperLimit(TouchSensor limitUpper){
        this.limitUpper = limitUpper;
    }

    // Set upper encoder limit
    public void setUpperLimit(double positionLimitUpper){
        this.positionLimitUpper = positionLimitUpper;
    }

    // Set upper and lower TouchSensor limits
    public void setLimits(TouchSensor limitLower, TouchSensor limitUpper){
        setLowerLimit(limitLower);
        setUpperLimit(limitUpper);
    }

    // Set upper and lower limits of any type

    public void setLimits(double positionLimitLower, double positionLimitUpper){
        setLowerLimit(positionLimitLower);
        setUpperLimit(positionLimitUpper);
    }

    public void setLimits(TouchSensor limitLower, double positionLimitUpper){
        setLowerLimit(limitLower);
        setUpperLimit(positionLimitUpper);
    }

    public void setLimits(double positionLimitLower, TouchSensor limitUpper){
        setLowerLimit(positionLimitLower);
        setUpperLimit(limitUpper);
    }

    // Check if either limit is pressed
    public boolean limitPressed(){
        return limitLowerPressed() || limitUpperPressed();
    }

    // Check whether a target position will trigger either limit
    public boolean limitPressed(double targetPosition){
        // Check the upper or lower limits, depending on which direction the motor is moving
        return targetPosition > getPosition() ? limitUpperPressed() : limitLowerPressed();
    }

    // Check whether the lower limit (encoder or TouchSensor) has been triggered
    public boolean limitLowerPressed(){
        if(limitLower == null){
            if(positionLimitLower == null){
                return false;
            }else{
                return getPosition() >= positionLimitLower;
            }
        }else{
            return limitLower.isPressed();
        }
    }

    // Check whether the upper limit (either encoder or TouchSensor) has been triggered
    public boolean limitUpperPressed(){
        if(limitUpper == null){
            if(positionLimitUpper == null){
                return false;
            }else{
                return getPosition() <= positionLimitUpper;
            }
        }else{
            return limitLower.isPressed();
        }
    }

    // DcMotorX methods with enforced limits

    public void goToDistance(double position, double speed){
        if(!core.getMode().equals(DcMotor.RunMode.RUN_TO_POSITION)) controlPosition();
        setDistance(position, speed);

        while(core.isBusy() && !limitPressed(position));
    }

    public void goToPosition(double position, double speed){
        if(!core.getMode().equals(DcMotor.RunMode.RUN_TO_POSITION)) controlPosition();
        setPosition(position, speed);

        while(core.isBusy() && !limitPressed(position));
    }

    // (Blocking) go to the lower limit
    public void goToLowerLimit(double speed){
        setPower(speed);
        while(!limitLower.isPressed());
        setPower(0);
    }

    // Reset methods just go to the lower limits
    public void reset(){ reset(0.2); }

    public void reset(double speed){
        goToLowerLimit(speed);
    }

    // (Blocking) go to the upper limit
    public void goToUpperLimit(double speed){
        setPower(speed);
        while(!limitUpper.isPressed());
        setPower(0);
    }

}
