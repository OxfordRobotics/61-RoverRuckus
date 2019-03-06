package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;


import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.robotcore.external.navigation.Position;
import org.firstinspires.ftc.robotcore.external.navigation.Velocity;

import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.ClassFactory;
import org.firstinspires.ftc.robotcore.external.matrices.OpenGLMatrix;
import org.firstinspires.ftc.robotcore.external.matrices.VectorF;
import org.firstinspires.ftc.robotcore.external.navigation.Acceleration;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackable;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackableDefaultListener;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackables;
import org.firstinspires.ftc.robotcore.external.tfod.Recognition;
import org.firstinspires.ftc.robotcore.external.tfod.TFObjectDetector;

import java.util.ArrayList;
import java.util.List;

import com.qualcomm.hardware.bosch.JustLoggingAccelerationIntegrator;


import static org.firstinspires.ftc.robotcore.external.navigation.AngleUnit.DEGREES;
import static org.firstinspires.ftc.robotcore.external.navigation.AxesOrder.XYZ;
import static org.firstinspires.ftc.robotcore.external.navigation.AxesOrder.YZX;
import static org.firstinspires.ftc.robotcore.external.navigation.AxesReference.EXTRINSIC;
import static org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer.CameraDirection.BACK;
import static org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer.CameraDirection.FRONT;

@Autonomous(name="Auto61IMU", group="Linear Opmode")

public class Auto61IMU extends LinearOpMode {

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

    BNO055IMU imu;
    Orientation angles;
    Acceleration gravity;
    Orientation             lastAngles = new Orientation();
    double                  globalAngle, power = .30, correction;

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

        targetsRoverRuckus.activate();

    } //Pre-operation code for vuforia to run, called before "waitForStart();"

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
                                if (goldMineralX < silverMineral1X && goldMineralX < silverMineral2X) {
                                    goldminerallocation = "left";
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

    @Override
    public void runOpMode() {
        angles   = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
        robot.init(hardwareMap);
        vuforiaInit();
        initTfod();
        robot.idolServo.setPosition(.4);
        robot.motorLift.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        robot.motorLift.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.motorLift.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
        parameters.angleUnit            = BNO055IMU.AngleUnit.DEGREES;
        parameters.accelUnit            = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
        parameters.calibrationDataFile  = "BNO055IMUCalibration.json";
        parameters.loggingEnabled       = true;
        parameters.loggingTag           = "IMU";
        parameters.accelerationIntegrationAlgorithm = new JustLoggingAccelerationIntegrator();


        imu = hardwareMap.get(BNO055IMU.class, "imu");
        imu.initialize(parameters);

        telemetry.addAction(new Runnable() { @Override public void run()
        {
            // Acquiring the angles is relatively expensive; we don't want
            // to do that in each of the three items that need that info, as that's
            // three times the necessary expense.
            angles   = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
            gravity  = imu.getGravity();
        }
        });

        telemetry.addData("Status", "Initialized");

        waitForStart();

        imu.startAccelerationIntegration(new Position(), new Velocity(), 1000);

        double start= angles.firstAngle;
        double destination;
        while (true){
            getVuforia();
            getTfod();
            telemetry.addData("location of the big gold boi", goldminerallocation);
            telemetry.addData("id of the big picture boys", visabletarget);
            telemetry.addData("Heading", angles.firstAngle);
            telemetry.addData("yayayeet", Double.parseDouble(runtime.toString().substring(0, 5)));
            telemetry.update();
            rotate(270, 1);
            telemetry.addData("location of the big gold boi", goldminerallocation);
            telemetry.addData("id of the big picture boys", visabletarget);
            telemetry.addData("Heading", angles.firstAngle);
            telemetry.addData("yayayeet", Double.parseDouble(runtime.toString().substring(0, 5)));
            telemetry.update();
            rotate(-270, 1);
        }


/*


        while (robot.motorLift.getCurrentPosition() < 19500) {
            robot.motorLift.setPower(1);
        }
        robot.motorLift.setPower(0);
        robot.motorLeft.setPower(.5);
        robot.motorRight.setPower(.5);
        sleep(120);
        robot.motorLeft.setPower(0);
        robot.motorRight.setPower(0);
        sleep(3000);

        if (goldminerallocation.equals("left")) {
            while (initialangle - angles.firstAngle < 72) {
                robot.motorLeft.setPower(-.5);
                robot.motorRight.setPower(.5);
            }
            robot.motorRight.setPower(0);
            robot.motorLeft.setPower(0);
            sleep(750);
            robot.motorRight.setPower(.5);
            robot.motorLeft.setPower(.5);
            sleep(875);
            robot.motorRight.setPower(0);
            robot.motorLeft.setPower(0);
            sleep(500);
            robot.motorRight.setPower(.5);
            robot.motorLeft.setPower(-.5);
            sleep(850);
            robot.motorRight.setPower(0);
            robot.motorLeft.setPower(0);
            sleep(500);
            robot.motorRight.setPower(.5);
            robot.motorLeft.setPower(.5);
            sleep(1000);
            robot.motorRight.setPower(0);
            robot.motorLeft.setPower(0);
            sleep(200);
//            robot.idolServo.setPosition(.1);
//            sleep(500);
//            robot.idolServo.setPosition(0);

        }

        else if (goldminerallocation.equals("right")) {
            robot.motorLeft.setPower(-.6);
            robot.motorRight.setPower(.5);
            sleep(1100);
            robot.motorRight.setPower(0);
            robot.motorLeft.setPower(0);
            sleep(750);
            robot.motorRight.setPower(.5);
            robot.motorLeft.setPower(.5);
            sleep(700);
            robot.motorRight.setPower(0);
            robot.motorLeft.setPower(0);
            sleep(200);
            robot.motorLeft.setPower(.5);
            robot.motorRight.setPower(-.6);
            sleep(1300); //was 2200
            robot.motorRight.setPower(.5);
            robot.motorLeft.setPower(.5);
            sleep(1000);
            robot.motorRight.setPower(0);
            robot.motorLeft.setPower(0);
            sleep(200);
//            robot.idolServo.setPosition(.1);
//            sleep(500);
//            robot.idolServo.setPosition(0);
        }

        else {
            robot.motorLeft.setPower(-.6);
            robot.motorRight.setPower(.5);
            sleep(800);
            robot.motorRight.setPower(0);
            robot.motorLeft.setPower(0);
            sleep(750);
            robot.motorRight.setPower(.25);
            robot.motorLeft.setPower(.25);
            sleep(4000);
            robot.motorRight.setPower(0);
            robot.motorLeft.setPower(0);
            sleep(200);
//            robot.idolServo.setPosition(.1);
//            sleep(500);
//            robot.idolServo.setPosition(0);
        }
*/
    }


    private void resetAngle()
    {
        lastAngles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);

        globalAngle = 0;
    }

    private double getAngle()
    {
        // We experimentally determined the Z axis is the axis we want to use for heading angle.
        // We have to process the angle because the imu works in euler angles so the Z axis is
        // returned as 0 to +180 or 0 to -180 rolling back to -179 or +179 when rotation passes
        // 180 degrees. We detect this transition and track the total cumulative angle of rotation.

        Orientation angles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);

        double deltaAngle = angles.firstAngle - lastAngles.firstAngle;

        if (deltaAngle < -180)
            deltaAngle += 360;
        else if (deltaAngle > 180)
            deltaAngle -= 360;

        globalAngle += deltaAngle;

        lastAngles = angles;

        return globalAngle;
    }

    private void rotate(int degrees, double power)
    {
        double  leftPower, rightPower;

        // restart imu movement tracking.
        resetAngle();

        // getAngle() returns + when rotating counter clockwise (left) and - when rotating
        // clockwise (right).

        if (degrees < 0)
        {   // turn right.
            leftPower = -power;
            rightPower = power;
        }
        else if (degrees > 0)
        {   // turn left.
            leftPower = power;
            rightPower = -power;
        }
        else return;

        // set power to rotate.
        robot.motorLeft.setPower(leftPower);
        robot.motorRight.setPower(rightPower);

        // rotate until turn is completed.
        if (degrees < 0)
        {
            // On right turn we have to get off zero first.
            while (opModeIsActive() && getAngle() == 0) {}

            while (opModeIsActive() && getAngle() > degrees) {}
        }
        else    // left turn.
            while (opModeIsActive() && getAngle() < degrees) {}

        // turn the motors off.
        robot.motorLeft.setPower(0);
        robot.motorRight.setPower(0);

        // wait for rotation to stop.
        sleep(1000);

        // reset angle tracking on new heading.
        resetAngle();
    }

}