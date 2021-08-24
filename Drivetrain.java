/*
Drivetrain Class
Provides easy control for a Mecanum drivetrain (feel free to modify this if using another system)
See https://www.notion.so/Drivetrain-3baa071900aa4ff6b7b192b2f50351c3 for documentation

Written by Isaac Krementsov, 12/7/2020
*/

package org.firstinspires.ftc.teamcode.api;

public class Drivetrain {

    // DcMotors represent each wheel
    public DcMotorX mRF, mLF, mRB, mLB;

    // Initialize a simple drivetrain with wheel motors
    public Drivetrain(DcMotorX mRF, DcMotorX mLF, DcMotorX mRB, DcMotorX mLB){
        this.mRF = mRF;
        this.mLF = mLF;
        this.mRB = mRB;
        this.mLB = mLB;

        reverseLeft();
        setBrake(true);
    }

    // Set power to both right wheels
    private void setPowerRight(double power){
        mRF.setPower(power);
        mRB.setPower(power);
    }

    // Set power to both left wheels
    private void setPowerLeft(double power){
        mLF.setPower(power);
        mLB.setPower(power);
    }

    // Set power to all wheels
    private void setPowerAll(double power){
        setPowerRight(power);
        setPowerLeft(power);
    }

    // Reverse all wheel motors
    public void reverse(){
        mRF.reverse();
        mRB.reverse();
        mLF.reverse();
        mLB.reverse();
    }

    // Reverse left wheel motors
    public void reverseLeft(){
        mLF.reverse();
        mLB.reverse();
    }

    // Base drive method, adjusted for use with gamepad joysticks
    public void driveWithGamepad(double speed, double forward, double yaw, double strafe){
        drive(speed*forward, -speed*yaw, speed*strafe);
    }

    /* Drive in a direction based on
        - Power: vertical/"forward"/parallel motion
        - Yaw: rotational motion, positive=counterclockwise
        - Strafe: horizontal/"sideways"/perpendicular motion
     */
    public void drive(double power, double yaw, double strafe){
        mRF.setPower(power + yaw - strafe);
        mLF.setPower(power - yaw + strafe);
        mRB.setPower(power + yaw + strafe);
        mLB.setPower(power - yaw - strafe);
    }

    // Drive in one direction
    public void drive(double power, Direction direction){
        switch(direction){
            case FORWARD:
                drive(power, 0, 0);
                break;
            case BACKWARD:
                drive(-power, 0, 0);
                break;
            case RIGHT:
                drive(0, 0, -power);
                break;
            case LEFT:
                drive(0, 0, power);
                break;
        }
    }

    // Set all motors to position control mode
    public void controlPosition(){
        mRF.controlPosition();
        mLF.controlPosition();
        mRB.controlPosition();
        mLB.controlPosition();
    }

    // Drive a set distance in one direction
    public void drive(double power, double distance, Direction direction, boolean blocking) {
        // Set the correct target distances based on drive direction
        switch(direction){
            case FORWARD:
                mRF.setTargetDistance(distance);
                mLF.setTargetDistance(distance);
                mRB.setTargetDistance(distance);
                mLB.setTargetDistance(distance);
            case BACKWARD:
                mRF.setTargetDistance(-distance);
                mLF.setTargetDistance(-distance);
                mRB.setTargetDistance(-distance);
                mLB.setTargetDistance(-distance);
            case RIGHT:
                mRF.setTargetDistance(-distance);
                mLF.setTargetDistance(distance);
                mRB.setTargetDistance(distance);
                mLB.setTargetDistance(-distance);
            case LEFT:
                mRF.setTargetDistance(distance);
                mLF.setTargetDistance(-distance);
                mRB.setTargetDistance(-distance);
                mLB.setTargetDistance(distance);
        }

        // Set the right motor powers
        drive(power, direction);

        // Wait for the motors to finish if blocking
        if(blocking){
            // Any of the motors can finish for the loop to stop
            while(mRF.core.isBusy() && mLF.core.isBusy() && mRB.core.isBusy() && mLB.core.isBusy());
        }
    }

    // Stop all motors
    public void stop(){
        setPowerAll(0);
    }

    // Change brake mode for all motors
    public void setBrake(boolean brake){
        mRF.setBrake(brake);
        mLF.setBrake(brake);
        mRB.setBrake(brake);
        mLB.setBrake(brake);
    }

    // Simple direction class, useful for very basic movements
    public static enum Direction {
        FORWARD, BACKWARD, RIGHT, LEFT
    }

}
