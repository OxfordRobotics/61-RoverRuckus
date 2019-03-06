package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;

public class HardwareMap
{
    public DcMotor  motorLeft = null;
    public DcMotor  motorRight = null;
    public DcMotor  motorLift = null;
    public DcMotor  extendo = null;
    public DcMotor  lifto = null;
    //public CRServo  colServo = null;
    public DcMotor    bucket = null;
    public Servo    idolServo = null;
    public CRServo  collecto = null;


    com.qualcomm.robotcore.hardware.HardwareMap hwMap = null;
    private ElapsedTime period  = new ElapsedTime();

    public HardwareMap(){

    }

    public void init(com.qualcomm.robotcore.hardware.HardwareMap ahwMap) {
        hwMap = ahwMap;

        motorLeft = hwMap.get(DcMotor.class, "motorLeft");
        motorRight = hwMap.get(DcMotor.class, "motorRight");
        motorLift = hwMap.get(DcMotor.class, "motorLift");
        lifto = hwMap.get(DcMotor.class, "lifto");
        extendo = hwMap.get(DcMotor.class, "extendo");
        //colServo = hwMap.get(CRServo.class, "colServo");
        bucket = hwMap.get(DcMotor.class, "bucket");
        idolServo = hwMap.get(Servo.class, "idolservo");
        collecto = hwMap.get(CRServo.class, "collecto");

        motorLeft.setDirection(DcMotor.Direction.REVERSE);
        motorRight.setDirection(DcMotor.Direction.FORWARD);
        motorLift.setDirection(DcMotor.Direction.REVERSE);
        extendo.setDirection(DcMotor.Direction.REVERSE);
        lifto.setDirection(DcMotor.Direction.FORWARD);
        collecto.setDirection(DcMotorSimple.Direction.REVERSE);

        motorLeft.setPower(0);
        motorRight.setPower(0);
        motorLift.setPower(0);
        extendo.setPower(0);
        lifto.setPower(0);

        idolServo.setPosition(0);

        motorLeft.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        motorRight.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        motorLift.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        //motorLift.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        extendo.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        lifto.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        lifto.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        lifto.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        bucket.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        bucket.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        lifto.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

    }
    public static double calibrate(double x){
        if( x > 180 )
            x -= 360;
        else if (x < -180 )
            x += 360;
        else
            x = x;
        return x;
    }
    public static double[] distance(double x,double y, String dir){
        double[] re = new double[10];
        double t;
        if(x*y>0) {
            t = Math.abs(x - y);
            if(x<y) {
                for (int i = 0; i < 10; i++) {
                    re[i] = (i + 1) * (t / 10.) + x;
                }
            }
            else{
                for(int i = 0; i<10; i++) {
                    re[i] = x -  (i + 1) * (t / 10.);
                }
            }
        }
        else if( dir.equals("left")){
            t = 180-Math.max(x, y) + Math.min(x,y)+180;
            if(x>y) {
                for (int i = 0; i < 10; i++) {
                    re[i] = calibrate((i + 1) * (t / 10.) + x);
                }
            }
            else{
                for(int i = 0; i<10; i++) {
                    re[i] = calibrate(x -  (i + 1) * (t / 10.));
                }
            }
        }
        else if (dir.equals("right")){
            t = Math.max(x, y) - Math.min(x,y)+180;
            if(x>y) {
                for (int i = 0; i < 10; i++) {
                    re[i] = calibrate((i + 1) * (t / 10.) + x);
                }
            }
            else{
                for(int i = 0; i<10; i++) {
                    re[i] = calibrate(x -  (i + 1) * (t / 10.));
                }
            }
        }
        else{

        }
        return re;
    }
}