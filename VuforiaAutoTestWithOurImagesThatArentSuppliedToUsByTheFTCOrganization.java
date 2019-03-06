package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.ClassFactory;
import org.firstinspires.ftc.robotcore.external.matrices.OpenGLMatrix;
import org.firstinspires.ftc.robotcore.external.matrices.VectorF;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackable;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackableDefaultListener;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackables;

import java.util.ArrayList;
import java.util.List;

import static org.firstinspires.ftc.robotcore.external.navigation.AngleUnit.DEGREES;
import static org.firstinspires.ftc.robotcore.external.navigation.AxesOrder.XYZ;
import static org.firstinspires.ftc.robotcore.external.navigation.AxesOrder.YZX;
import static org.firstinspires.ftc.robotcore.external.navigation.AxesReference.EXTRINSIC;
import static org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer.CameraDirection.BACK;
import static org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer.CameraDirection.FRONT;

@TeleOp(name="vuforiatestbutnotdefaultpicsanditsjustthecube", group="Linear Opmode")
@Disabled
public class VuforiaAutoTestWithOurImagesThatArentSuppliedToUsByTheFTCOrganization extends LinearOpMode {

    private static final String VUFORIA_KEY = "YOUR KEY HERE";

    private static final float mmPerInch        = 25.4f;
    private static final float mmFTCFieldWidth  = (12*6) * mmPerInch;       // the width of the FTC field (from the center point to the outer panels)
    private static final float mmTargetHeight   = (6) * mmPerInch;          // the height of the center of the target image above the floor

    private static final VuforiaLocalizer.CameraDirection CAMERA_CHOICE = BACK;

    private OpenGLMatrix lastLocation = null;
    private boolean targetVisible = false;

    VuforiaLocalizer vuforia;

    public List<VuforiaTrackable> allTrackables = new ArrayList<VuforiaTrackable>();

    double xdis = 0;
    double ydis = 0;
    double zdis = 0;
    double roll = 0;
    double pitch = 0;
    double heading = 0;

    HardwareMap hwMap = new HardwareMap();
    private ElapsedTime runtime = new ElapsedTime();

    void vuforiaInit() {
        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        VuforiaLocalizer.Parameters parameters = new VuforiaLocalizer.Parameters(cameraMonitorViewId);

        parameters.vuforiaLicenseKey = VUFORIA_KEY;
        parameters.cameraDirection = CAMERA_CHOICE;

        vuforia = ClassFactory.getInstance().createVuforia(parameters);

        VuforiaTrackables targetsRoverRuckus = this.vuforia.loadTrackablesFromAsset("roboticscubesandcirclesexcepttheresnocircles");
        VuforiaTrackable blueRover = targetsRoverRuckus.get(0);
        blueRover.setName("literallytheonlyimage");

        allTrackables.addAll(targetsRoverRuckus);

        OpenGLMatrix blueRoverLocationOnField = OpenGLMatrix
                .translation(0, mmFTCFieldWidth, mmTargetHeight)
                .multiplied(Orientation.getRotationMatrix(EXTRINSIC, XYZ, DEGREES, 90, 0, 0));
        blueRover.setLocation(blueRoverLocationOnField);

        final int CAMERA_FORWARD_DISPLACEMENT = 0;   // eg: Camera is 110 mm in front of robot center
        final int CAMERA_VERTICAL_DISPLACEMENT = 0;   // eg: Camera is 200 mm above ground
        final int CAMERA_LEFT_DISPLACEMENT = 0;     // eg: Camera is ON the robot's center line

        OpenGLMatrix phoneLocationOnRobot = OpenGLMatrix
                .translation(CAMERA_FORWARD_DISPLACEMENT, CAMERA_LEFT_DISPLACEMENT, CAMERA_VERTICAL_DISPLACEMENT)
                .multiplied(Orientation.getRotationMatrix(EXTRINSIC, YZX, DEGREES,
                        CAMERA_CHOICE == FRONT ? 90 : -90, 0, 0));

        for (VuforiaTrackable trackable : allTrackables) {
            ((VuforiaTrackableDefaultListener) trackable.getListener()).setPhoneInformation(phoneLocationOnRobot, parameters.cameraDirection);
        }

        telemetry.addData(">", "Press Play to start tracking");
        telemetry.update();
        waitForStart();

        targetsRoverRuckus.activate();

    }

    void getVuforia() {
        targetVisible = false;
        for (VuforiaTrackable trackable : allTrackables) {
            if (((VuforiaTrackableDefaultListener)trackable.getListener()).isVisible()) {
                telemetry.addData("Visible Target", trackable.getName());
                targetVisible = true;

                OpenGLMatrix robotLocationTransform = ((VuforiaTrackableDefaultListener)trackable.getListener()).getUpdatedRobotLocation();
                if (robotLocationTransform != null) {
                    lastLocation = robotLocationTransform;
                }
                break;
            }
        }

        if (targetVisible) {
            VectorF translation = lastLocation.getTranslation();
            telemetry.addData("Pos (in)", "{X, Y, Z} = %.1f, %.1f, %.1f",
                    translation.get(0) / mmPerInch, translation.get(1) / mmPerInch, translation.get(2) / mmPerInch);

            xdis = translation.get(0) / mmPerInch;
            ydis = translation.get(1) / mmPerInch;
            zdis = translation.get(2) / mmPerInch;

            Orientation rotation = Orientation.getOrientation(lastLocation, EXTRINSIC, XYZ, DEGREES);
            telemetry.addData("Rot (deg)", "{Roll, Pitch, Heading} = %.0f, %.0f, %.0f", rotation.firstAngle, rotation.secondAngle, rotation.thirdAngle);

            roll = rotation.firstAngle;
            pitch = rotation.secondAngle;
            heading = rotation.thirdAngle;

        }
        else {
            telemetry.addData("Visible Target", "none");
            xdis = 0;
            ydis = 0;
            zdis = 0;
            roll = 0;
            pitch = 0;
            heading = 0;
        }
        telemetry.update();
    }



    @Override
    public void runOpMode() {

        telemetry.addData("Status", "Initialized");
        hwMap.init(hardwareMap);
        telemetry.addData("Status", "Initialized");

        telemetry.addData("Status", "Run Time: " + runtime.toString());

        vuforiaInit();

        /*while (Double.parseDouble(runtime.toString().substring(0, 7)) < 10) {
            getVuforia();
        }*/

        while (ydis < 50 || ydis == 0) {
            getVuforia();
        }

        telemetry.addData("exited", "actually works btw");

        telemetry.update();

        sleep(5000);

    }

}