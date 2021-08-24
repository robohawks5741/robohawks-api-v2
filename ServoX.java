 /*
Extended Servo API
Visit https://www.notion.so/ServoX-86d5734b7f2d412aa7bf537137e3ba24 for documentation
*/

package org.firstinspires.ftc.teamcode.api;

import com.qualcomm.robotcore.hardware.Servo;

public class ServoX {

    public Servo core;

    // Total rotational range of the servo (this can include angles outside of min/max)
    public double rotationAngle;
    // Maximum allowable position
    public double maxAngle = Double.MAX_VALUE;
    // Minimum allowable position
    public double minAngle = Double.MIN_VALUE;

    // Simple 180 degree servo initialization
    public ServoX(Servo core){ this(core, 180, 180); }
    // 0-x degree servo initialization with custom rotational range
    public ServoX(Servo core, double rotationAngle, double maxAngle){ this(core, rotationAngle, maxAngle, 0); }

    // x-y degree servo initialization with custom rotational range
    public ServoX(Servo core, double rotationAngle, double maxAngle, double minAngle){
        this.rotationAngle = rotationAngle;
        this.maxAngle = maxAngle;
        this.minAngle = minAngle;
        this.core = core;
    }

    // Increase/decrease the target angle
    public void setDistance(double angle){
        setAngle(angle + getAngle());
    }

    // Increase/decrease servo position
    public void goToDistance(double angle, int wait) throws InterruptedException {
        goToAngle(angle, wait);
    }

     // Set target angle and wait for the servo to actually move
    public void goToAngle(double angle, int wait) throws InterruptedException {
        setAngle(angle);

        if(wait > 0){
            Thread.sleep(wait);
        }
    }

    // Set the target angle
    public void setAngle(double angle) {
        // Make sure angle is within acceptable bounds
        if(angle <= maxAngle && angle >= minAngle) {
            // Convert angle to 0-1 scale
            core.setPosition(angle / rotationAngle);
        }
    }

    // Get current servo angle
    public double getAngle(){
        return core.getPosition() * rotationAngle;
    }

}
