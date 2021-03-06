package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.ClassFactory;
import org.firstinspires.ftc.robotcore.external.matrices.OpenGLMatrix;
import org.firstinspires.ftc.robotcore.external.matrices.VectorF;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackable;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackableDefaultListener;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackables;
import org.firstinspires.ftc.robotcore.external.tfod.Recognition;
import org.firstinspires.ftc.robotcore.external.tfod.TFObjectDetector;

import java.util.ArrayList;
import java.util.List;

import static org.firstinspires.ftc.robotcore.external.navigation.AngleUnit.DEGREES;
import static org.firstinspires.ftc.robotcore.external.navigation.AxesOrder.XYZ;
import static org.firstinspires.ftc.robotcore.external.navigation.AxesOrder.YZX;
import static org.firstinspires.ftc.robotcore.external.navigation.AxesReference.EXTRINSIC;
import static org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer.CameraDirection.BACK;
import static org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer.CameraDirection.FRONT;

@Autonomous(name="Auto61DepotEncoders", group="Linear Opmode")
public class Auto61DepotEncoders extends LinearOpMode {

    private static final String VUFORIA_KEY = "YOUR KEY HERE";

    private static final float mmPerInch        = 25.4f;
    private static final float mmFTCFieldWidth  = (12*6) * mmPerInch;       // the width of the FTC field (from the center point to the outer panels)
    private static final float mmTargetHeight   = (6) * mmPerInch;          // the height of the center of the target image above the floor

    private static final VuforiaLocalizer.CameraDirection CAMERA_CHOICE = BACK;

    private OpenGLMatrix lastLocation = null;
    private boolean targetVisible = false;

    VuforiaLocalizer vuforia;

    public List<VuforiaTrackable> allTrackables = new ArrayList<VuforiaTrackable>();

    private static final String TFOD_MODEL_ASSET = "RoverRuckus.tflite";
    private static final String LABEL_GOLD_MINERAL = "Gold Mineral";
    private static final String LABEL_SILVER_MINERAL = "Silver Mineral";

    private TFObjectDetector tfod;

    double xdis = 0;
    double ydis = 0;
    double zdis = 0;
    double roll = 0;
    double pitch = 0;
    double heading = 0;
    String visabletarget = "none";
    String goldminerallocation = "none";

    HardwareMap robot = new HardwareMap();
    private ElapsedTime runtime = new ElapsedTime();

    void vuforiaInit() {
        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        VuforiaLocalizer.Parameters parameters = new VuforiaLocalizer.Parameters(cameraMonitorViewId);

        parameters.vuforiaLicenseKey = VUFORIA_KEY;
        parameters.cameraDirection = VuforiaLocalizer.CameraDirection.BACK;

        vuforia = ClassFactory.getInstance().createVuforia(parameters);

        VuforiaTrackables targetsRoverRuckus = this.vuforia.loadTrackablesFromAsset("RoverRuckus");
        VuforiaTrackable blueRover = targetsRoverRuckus.get(0);
        blueRover.setName("Blue-Rover");
        VuforiaTrackable redFootprint = targetsRoverRuckus.get(1);
        redFootprint.setName("Red-Footprint");
        VuforiaTrackable frontCraters = targetsRoverRuckus.get(2);
        frontCraters.setName("Front-Craters");
        VuforiaTrackable backSpace = targetsRoverRuckus.get(3);
        backSpace.setName("Back-Space");

        allTrackables.addAll(targetsRoverRuckus);

        OpenGLMatrix blueRoverLocationOnField = OpenGLMatrix
                .translation(0, mmFTCFieldWidth, mmTargetHeight)
                .multiplied(Orientation.getRotationMatrix(EXTRINSIC, XYZ, DEGREES, 90, 0, 0));
        blueRover.setLocation(blueRoverLocationOnField);

        OpenGLMatrix redFootprintLocationOnField = OpenGLMatrix
                .translation(0, -mmFTCFieldWidth, mmTargetHeight)
                .multiplied(Orientation.getRotationMatrix(EXTRINSIC, XYZ, DEGREES, 90, 0, 180));
        OpenGLMatrix frontCratersLocationOnField = OpenGLMatrix
                .translation(-mmFTCFieldWidth, 0, mmTargetHeight)
                .multiplied(Orientation.getRotationMatrix(EXTRINSIC, XYZ, DEGREES, 90, 0, 90));
        frontCraters.setLocation(frontCratersLocationOnField);

        OpenGLMatrix backSpaceLocationOnField = OpenGLMatrix
                .translation(mmFTCFieldWidth, 0, mmTargetHeight)
                .multiplied(Orientation.getRotationMatrix(EXTRINSIC, XYZ, DEGREES, 90, 0, -90));
        backSpace.setLocation(backSpaceLocationOnField);

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

        targetsRoverRuckus.activate();

    } //Pre-operation code for vuforia and TensorFlow to run, should be called before "waitForStart();"

    void getVuforia() {
        targetVisible = false;
        for (VuforiaTrackable trackable : allTrackables) {
            if (((VuforiaTrackableDefaultListener)trackable.getListener()).isVisible()) {
                targetVisible = true;
                visabletarget = trackable.getName();

                OpenGLMatrix robotLocationTransform = ((VuforiaTrackableDefaultListener)trackable.getListener()).getUpdatedRobotLocation();
                if (robotLocationTransform != null) {
                    lastLocation = robotLocationTransform;
                }
                break;
            }
        }

        if (targetVisible) {
            VectorF translation = lastLocation.getTranslation();

            xdis = translation.get(0) / mmPerInch;
            ydis = translation.get(1) / mmPerInch;
            zdis = translation.get(2) / mmPerInch;


            Orientation rotation = Orientation.getOrientation(lastLocation, EXTRINSIC, XYZ, DEGREES);
            roll = rotation.firstAngle;
            pitch = rotation.secondAngle;
            heading = rotation.thirdAngle;

        }
        else {
            xdis = 0;
            ydis = 0;
            zdis = 0;
            roll = 0;
            pitch = 0;
            heading = 0;
            visabletarget = "none";
        }
        telemetry.update();
    }

    private void initTfod() {
        int tfodMonitorViewId = hardwareMap.appContext.getResources().getIdentifier(
                "tfodMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        TFObjectDetector.Parameters tfodParameters = new TFObjectDetector.Parameters(tfodMonitorViewId);
        tfod = ClassFactory.getInstance().createTFObjectDetector(tfodParameters, vuforia);
        tfod.loadModelFromAsset(TFOD_MODEL_ASSET, LABEL_GOLD_MINERAL, LABEL_SILVER_MINERAL);

        if (tfod != null) {
            tfod.activate();
        }
    } //Pre-operation

    private void getTfod() {
            if (tfod != null) {
                // getUpdatedRecognitions() will return null if no new information is available since
                // the last time that call was made.
                List<Recognition> updatedRecognitions = tfod.getUpdatedRecognitions();
                if (updatedRecognitions != null) {
                    if (updatedRecognitions.size() == 2 || updatedRecognitions.size() == 3) {
                        int goldMineralX = -1;
                        int silverMineral1X = -1;
                        int silverMineral2X = -1;
                        for (Recognition recognition : updatedRecognitions) {
                            if (recognition.getLabel().equals(LABEL_GOLD_MINERAL)) {
                                goldMineralX = (int) recognition.getLeft();
                            } else if (silverMineral1X == -1) {
                                silverMineral1X = (int) recognition.getLeft();
                            } else {
                                silverMineral2X = (int) recognition.getLeft();
                            }
                        }
                        if (Math.max(Math.max(goldMineralX, silverMineral1X), silverMineral2X) == goldMineralX) goldMineralX = -1;
                        else if (Math.max(Math.max(goldMineralX, silverMineral1X), silverMineral2X) == silverMineral1X) silverMineral1X = -1;
                        else if (Math.max(Math.max(goldMineralX, silverMineral1X), silverMineral2X) == silverMineral2X) silverMineral2X = -1;
                        if ((goldMineralX != -1 && silverMineral1X != -1) || (silverMineral1X != -1 && silverMineral2X != -1)) {// && updatedRecognitions.size() == 2) {
                                telemetry.addData("recog length", 2);
                                telemetry.update();
                                if ((goldMineralX < silverMineral1X || goldMineralX < silverMineral2X) && goldMineralX != -1) {
                                    goldminerallocation = "left";
                                } else if (silverMineral2X >= 0) {
                                    goldminerallocation = "right";
                                } else {
                                    goldminerallocation = "center";
                                }
                            }
                            /* else if (goldMineralX != -1 && silverMineral1X != -1 && silverMineral2X != -1 && updatedRecognitions.size() == 3){
                                telemetry.addData("recog length", 3);
                                telemetry.update();
                                    goldminerallocation = "left";
                                if (goldMineralX < silverMineral1X && goldMineralX < silverMineral2X) {
                                } else if (goldMineralX > silverMineral1X && goldMineralX > silverMineral2X) {
                                    goldminerallocation = "right";
                                } else {
                                    goldminerallocation = "center";
                                }

                            } */
                    }
                    telemetry.update();
                }
            }
        }

    private double inchToEncoder( int inches ) {
        return inches / ( 4 * Math.PI ) * 1120; // 4 is the wheel diameter in inches, 1120 is how much the encoders count per one output rotation
    }
    @Override
    public void runOpMode() {
        robot.init(hardwareMap);
        vuforiaInit();
        initTfod();

        robot.idolServo.setPosition(.4);
        robot.motorLift.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        robot.motorLift.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.motorLift.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        robot.motorLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        robot.motorLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.motorLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        robot.motorRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        robot.motorRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.motorRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        robot.motorLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        robot.motorRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        waitForStart();

        runtime.reset();

        sleep(1500);
        while (robot.motorLift.getCurrentPosition() < 19250) {
            robot.motorLift.setPower(1);
        }

        robot.motorLift.setPower(0);

        while (goldminerallocation.equals("none") && Double.parseDouble(runtime.toString().substring(0, 5)) < 10) {
            getVuforia();
            getTfod();
            telemetry.addData("location of the big gold boi", goldminerallocation);
            telemetry.addData("id of the big picture boys", visabletarget);
            telemetry.addData("time", Double.parseDouble(runtime.toString().substring(0, 5)));
            telemetry.update();
        }

        sleep(500);

        robot.motorRight.setPower(.2);
        robot.motorLeft.setPower(.2);
        sleep(330);
        robot.motorLeft.setPower(0);
        robot.motorRight.setPower(0);
        sleep(2000);
        robot.motorLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        robot.motorRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        robot.motorRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.motorRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        if (goldminerallocation.equals("right")) {
            while (robot.motorLeft.getCurrentPosition() > -240 &&  robot.motorRight.getCurrentPosition() < 240) {
                robot.motorRight.setPower(.2);
                robot.motorLeft.setPower(-.2);
            }
            while (robot.motorLeft.getCurrentPosition() < 1400 &&  robot.motorRight.getCurrentPosition() < 1400) {
                robot.motorRight.setPower(.5);
                robot.motorLeft.setPower(.2);
            }
            while (robot.motorLeft.getCurrentPosition() < 700 &&  robot.motorRight.getCurrentPosition() < 1000) {
                robot.motorRight.setPower(.5);
                robot.motorLeft.setPower(.2);
            }
        }
        else if (goldminerallocation.equals("center")) {
            while (robot.motorLeft.getCurrentPosition() > -180 &&  robot.motorRight.getCurrentPosition() < 180) {
                robot.motorRight.setPower(.2);
                robot.motorLeft.setPower(-.2);
            }
            while (robot.motorLeft.getCurrentPosition() < 1400 &&  robot.motorRight.getCurrentPosition() < 1400) {
                robot.motorRight.setPower(.5);
                robot.motorLeft.setPower(.2);
            }
            while (robot.motorLeft.getCurrentPosition() < 700 &&  robot.motorRight.getCurrentPosition() < 1000) {
                robot.motorRight.setPower(.65);
                robot.motorLeft.setPower(.2);
            }

        }
        else {
            while (robot.motorLeft.getCurrentPosition() > -50 &&  robot.motorRight.getCurrentPosition() < 50) {
                robot.motorRight.setPower(.1);
                robot.motorLeft.setPower(-.1);
            }

            robot.motorLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            robot.motorRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            robot.motorRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            robot.motorRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            while (robot.motorLeft.getCurrentPosition() < 1400 &&  robot.motorRight.getCurrentPosition() < 1400) {
                robot.motorRight.setPower(.325);
                robot.motorLeft.setPower(.1);
            }
            while (robot.motorLeft.getCurrentPosition() < 700 &&  robot.motorRight.getCurrentPosition() < 1000) {
                robot.motorRight.setPower(.325);
                robot.motorLeft.setPower(.1);
            }
            }
            robot.motorLeft.setPower(0);
            robot.motorRight.setPower(0);
            robot.motorLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            robot.motorRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            robot.motorRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);


            robot.motorLeft.setPower(0);
            robot.motorRight.setPower(0);


            if (tfod != null) {
                tfod.shutdown();
            }

        }
    }
