/*
Extended TensorFlow/Vuforia API
Visit https://www.notion.so/TensorFlowX-2ad482f85c4f4e72b839136eead80bdd for documentation
*/

package org.firstinspires.ftc.teamcode.api;

import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.ClassFactory;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import org.firstinspires.ftc.robotcore.external.tfod.Recognition;
import org.firstinspires.ftc.robotcore.external.tfod.TFObjectDetector;
import org.firstinspires.ftc.teamcode.R;

import java.util.ArrayList;
import java.util.List;

public class TensorFlowX {

    // Vuforia developer key
    private String vuforiaKey;
    // The camera (front or back) for Vuforia to use
    private VuforiaLocalizer.CameraDirection camera;

    // Asset file to use for Tfod model
    private String tfodModelAsset;
    private int tfodMonitorViewID;
    // Labels that can be output by the model
    private String[] labels;

    public VuforiaLocalizer vuforia;
    public TFObjectDetector tfod;

    // Initialize TensorFlowX from the model path, Vuforia key, camera direction, labels, and hardwareMap
    public TensorFlowX(String tfodModelAsset, String vuforiaKey, VuforiaLocalizer.CameraDirection camera, String[] labels, HardwareMap hardwareMap) throws Exception {
        this.tfodModelAsset = tfodModelAsset;
        this.vuforiaKey = vuforiaKey;
        this.camera = camera;
        this.labels = labels;

        tfodMonitorViewID = hardwareMap.appContext.getResources().getIdentifier(
                "tfodMonitorViewId",
                "id",
                hardwareMap.appContext.getPackageName()
        );

        init();
    }

    // Initialize Vuforia
    private void initVuforia(){
        VuforiaLocalizer.Parameters params = new VuforiaLocalizer.Parameters();

        params.vuforiaLicenseKey = vuforiaKey;
        params.cameraDirection = camera;

        // Plug license key and camera direction into a new Vuforia instance
        vuforia = ClassFactory.getInstance().createVuforia(params);
    }

    // Initialize Tensorflow
    private void initTfod(){
        TFObjectDetector.Parameters params = new TFObjectDetector.Parameters(tfodMonitorViewID);
        // Ignore result with confidence below 80% (this can be changed as needed)
        params.minResultConfidence = 0.8f;

        // Create TFOD instance and load model
        tfod = ClassFactory.getInstance().createTFObjectDetector(params, vuforia);
        tfod.loadModelFromAsset(tfodModelAsset, labels);
    }

    // Try to initialize and activate Vuforia/TensorFlow
    private void init() throws Exception {
        initVuforia();
        initTfod();

        if(tfod != null){
            tfod.activate();
        }else{
            throw new Exception("This device is not compatible with TFOD");
        }
    }

    // Shut the object detector down
    public void shutdown(){
        tfod.shutdown();
    }

    // Look for a single detection with given label
    public Recognition recognize(String label){
        Recognition matched = null;
        // Get all recognitions
        List<Recognition> recognitions = getRecognitions();

        // Search list of recognitions for one that matches the label
        for(Recognition recognition : recognitions){
            if(recognition.getLabel().equals(label)){
                matched = recognition;
                break;
            }
        }

        return matched;
    }

    // Find all matches for a given label
    public List<Recognition> recognizeAll(String label){
        List<Recognition> matched = new ArrayList<>();
        // Get all recognitions
        List<Recognition> recognitions = getRecognitions();

        // Search recognition list for all matches
        for(Recognition recognition : recognitions){
            if(recognition.getLabel().equals(label)){
                matched.add(recognition);
            }
        }

        return matched;
    }

    // Get all updated & old recognitions
    public List<Recognition> getRecognitions(){
        // Get latest recognitions
        List<Recognition> recs = tfod.getUpdatedRecognitions();
        if(recs == null) recs = new ArrayList();

        // Merge old recognitions
        List<Recognition> oldRecs = tfod.getRecognitions();
        if(oldRecs != null) recs.addAll(oldRecs);

        return recs;
    }

}
