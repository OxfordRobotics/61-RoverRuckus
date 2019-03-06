package org.firstinspires.ftc.teamcode;

import android.provider.OpenableColumns;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;

import com.vuforia.HINT;
import com.vuforia.Vuforia;

import org.firstinspires.ftc.robotcore.external.ClassFactory;
import org.firstinspires.ftc.robotcore.external.matrices.OpenGLMatrix;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackable;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackableDefaultListener;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackables;

@TeleOp(name="VuforiaTeleTest", group="Iterative Opmode")
@Disabled
public class VuforiaTeleTest extends OpMode {

    private ElapsedTime runtime = new ElapsedTime();

    VuforiaLocalizer vuforiaLocalizer;
    VuforiaLocalizer.Parameters parameters;
    VuforiaTrackables visionTargets;
    VuforiaTrackable target;
    VuforiaTrackableDefaultListener listener;

    OpenGLMatrix lastKnownLocation;
    OpenGLMatrix phoneLocation;

    private static final String VUFORIA_KEY = "YOUR KEY HERE";

     //Code to run ONCE when the driver hits INIT
    @Override
    public void init() {
        telemetry.addData("Status", "Initialized");
    }


     //Code to run REPEATEDLY after the driver hits INIT, but before they hit PLAY
    @Override
    public void init_loop() {
    }


     //Code to run ONCE when the driver hits PLAY
    @Override
    public void start() {
        runtime.reset();
    }


     //Code to run REPEATEDLY after the driver hits PLAY but before they hit STOP
    @Override
    public void loop() {


        telemetry.addData("Status", "Run Time: " + runtime.toString());
    }


     //Code to run ONCE after the driver hits STOP
    @Override
    public void stop() {


    }

    public void setupVuforia() {

        // Setup parameters to create localizer
        parameters = new VuforiaLocalizer.Parameters(R.id.cameraMonitorViewId); // To remove the camera view from the screen, remove the R.id.cameraMonitorViewId
        parameters.vuforiaLicenseKey = VUFORIA_KEY;
        parameters.cameraDirection = VuforiaLocalizer.CameraDirection.BACK;
        parameters.useExtendedTracking = false;
        vuforiaLocalizer = ClassFactory.createVuforiaLocalizer(parameters);

        // These are the vision targets that we want to use
        // The string needs to be the name of the appropriate .xml file in the assets folder
        visionTargets = vuforiaLocalizer.loadTrackablesFromAsset("RoverRuckus");
        Vuforia.setHint(HINT.HINT_MAX_SIMULTANEOUS_IMAGE_TARGETS, 4);

        // Setup the target to be tracked
        target = visionTargets.get(0); // 0 corresponds to the wheels target
        target.setName("BluePerimeter");
       // target.setLocation(createMatrix(0, 500, 0, 90, 0, 90));

        // Set phone location on robot
      //  phoneLocation = createMatrix(0, 225, 0, 90, 0, 0);

        // Setup listener and inform it of phone information
        listener = (VuforiaTrackableDefaultListener) target.getListener();
        listener.setPhoneInformation(phoneLocation, parameters.cameraDirection);

    }

}