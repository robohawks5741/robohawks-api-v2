/*
Extended DcMotor API
Visit https://www.notion.so/DcMotorX-6f78a2bd9f144c5791de8fef8621e37e for documentation
*/

package org.firstinspires.ftc.teamcode.api;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

public class DcMotorX {

    public DcMotor core;
    // Encoder ticks per motor revolution
    public int ticksPerRev;
    // Distance traveled by robot/mechanism per revolution
    public double distancePerRev;

    // Temporary variables for tracking changes in position
    private double lastPosition = 0;
    private double currentPosition = 0;

    // Initialize motor without using encoders
    public DcMotorX(DcMotor motor){
        this(motor, 0, 0);
    }

    // Initialize motor with basic encoder settings
    public DcMotorX(DcMotor motor, int ticksPerRev, double distancePerRev){
        this.core = motor;
        this.ticksPerRev = ticksPerRev;
        this.distancePerRev = distancePerRev;
    }

    // Get encoder ticks for a certain distance
    private int getEncoderPosition(double distance){
        return (int) Math.round(ticksPerRev * distance / distancePerRev);
    }

    // Get motor displacement from encoder ticks (inverse of getEncoderPosition)
    private double getDistanceFrom(int encoderPosition){
        return distancePerRev * encoderPosition / ticksPerRev;
    }

    // Set motor to run at a PID-controlled constant velocity
    public void controlVelocity(){
        core.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }

    // Set motor to run to positions with built-in PID controller
    public void controlPosition(){
        core.setTargetPosition(0);
        core.setMode(DcMotor.RunMode.RUN_TO_POSITION);
    }

    // Reset the motor's encoder
    public void resetEncoder(){
        core.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
    }

    // Set the motor run at a set power, regardless of encoder input
    public void runWithoutEncoder(){
        core.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
    }

    // Switch motor's default direction
    public void reverse(){
        DcMotorSimple.Direction direction = core.getDirection();

        if(direction.equals(DcMotorSimple.Direction.REVERSE)){
            core.setDirection(DcMotorSimple.Direction.FORWARD);
        }else{
            core.setDirection(DcMotorSimple.Direction.REVERSE);
        }
    }

    // Get motor power (use in runWithoutEncoder mode)
    public double getPower(){
        return core.getPower();
    }

    // Get motor velocity (use in controlVelocity mode)
    public double getVelocity() { return getPower(); }

    // Set motor power (use in runWithoutEncoder mode)
    public void setPower(double power){
        core.setPower(power);
    }

    // Get current motor position
    public double getPosition(){
        currentPosition = getDistanceFrom(core.getCurrentPosition());
        return currentPosition;
    }

    // Get motor position with direction swapped if reversed
    public double getPositionForward(){
        if(core.getDirection() == DcMotorSimple.Direction.REVERSE) return -getPosition();
        else return getPosition();
    }

    // Save current position for measuring displacement
    public void savePosition(boolean useCurrent){
        if(useCurrent) lastPosition = currentPosition;
        else lastPosition = getPosition();
    }

    // Get motor displacement from a saved position
    public double getDistance(boolean useCurrent){
        if(useCurrent) return currentPosition - lastPosition;
        else return getPosition() - lastPosition;
    }

    // Set constant velocity to run at (use in controlVelocity mode)
    public void setVelocity(double velocity){
        core.setPower(velocity);
    }

    // Set position to run to, at a particular speed (use in controlPosition mode, will start traveling to position)
    public void setPosition(double position, double speed){
        core.setTargetPosition(getEncoderPosition(position));
        core.setPower(speed);
    }

    // Set target displacement from current position (use in controlPosition mode, will start traveling to position)
    public void setDistance(double distance, double speed){
        setPosition(getPosition() + distance, speed);
    }

    // Set target position and travel to it
    public void goToPosition(double position, double speed){
        // Set motor to controlPosition mode if this hasn't been done already
        if(!core.getMode().equals(DcMotor.RunMode.RUN_TO_POSITION)) controlPosition();
        setPosition(position, speed);

        // isBusy returns false once core.getPosition() ≈≈ core.getTargetPosition()
        while(core.isBusy());
    }

    // Set target displacement and travel to it
    public void goToDistance(double distance, double speed){
        goToPosition(distance + getPosition(), speed);
    }

    // Use custom controller to get a target speed (useful for some encoder-based arm mechanisms)
    public static double getControlledSpeed(double maxSpeed, double threshold, double error, boolean useThreshold){
        double speed = Math.signum(error)*maxSpeed;

        if(Math.abs(error) < 2*threshold){
            speed *= error / (2*threshold);
        }

        if(Math.abs(error) < threshold && useThreshold){
            speed = 0;
        }

        return speed;
    }

    // Get current target position
    public double getTargetPosition(){
        return getDistanceFrom(core.getTargetPosition());
    }

    // Set the motor to either float or brake when given zero power
    public void setBrake(boolean brake){
        if(brake){
            core.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        }else{
            core.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        }
    }

    // Set a target position for the motor
    protected void setTargetPosition(double targetPosition){ core.setTargetPosition(getEncoderPosition(targetPosition)); }

    // Set a target displacement for the motor
    protected void setTargetDistance(double targetDistance){ setTargetPosition(getPosition() + targetDistance); }
}
