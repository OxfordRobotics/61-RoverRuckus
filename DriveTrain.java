package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.DcMotor;

//Wheel to wheel: 13.625 in
//Wheel diameter: 4 in
//Encoders per wheel rotation: 1120
//Motor max rpm: 129

public class DriveTrain {
    DcMotor leftMotor;
    DcMotor rightMotor;

    public DriveTrain( DcMotor newLeftMotor, DcMotor newRightMotor ) {
        leftMotor = newLeftMotor;
        rightMotor = newRightMotor;
    }

    public DcMotor getLeftMotor() { return leftMotor; }
    public DcMotor getRightMotor() { return rightMotor; }
    public void setLeftMotor(DcMotor leftMotor) { this.leftMotor = leftMotor; }
    public void setRightMotor(DcMotor rightMotor) { this.rightMotor = rightMotor; }

    private int inchToEncoder(double inch ) {
        long encoders = Math.round(inch / ( 4 * Math.PI ) * 1120);  // 4 is the wheel diameter in inches, 1120 is how much the encoders count per one output rotation
        if (encoders < Integer.MAX_VALUE && encoders > Integer.MIN_VALUE)
            return (int)encoders;
        return 0;
    }

    private void sleepPower( double power, double inch ) {
        try {
            Thread.sleep( Math.round( ( 1 / ( 129 / 60000.0 * Math.abs(power) ) ) * ( Math.abs(inch) / ( 4 * Math.PI ) ) ) + 350 );
        }
        catch(InterruptedException ex) {
            Thread.currentThread().interrupt();
        }
    }

    private void sleep( long millis ) {
        try {
            Thread.sleep( millis );
        }
        catch(InterruptedException ex) {
            Thread.currentThread().interrupt();
        }
    }

    public void go( double inch, double power ) {
        leftMotor.setTargetPosition( leftMotor.getCurrentPosition() + ( inchToEncoder(inch) * -1 ) );
        rightMotor.setTargetPosition( rightMotor.getCurrentPosition() + ( inchToEncoder(inch) * -1 ) );
        leftMotor.setPower( power );
        rightMotor.setPower( power );
        while (leftMotor.isBusy() || rightMotor.isBusy()) {
        }
        sleep( 200 );
    }

    public void turnOneLeft( double angle, double power ) {
        leftMotor.setTargetPosition( leftMotor.getCurrentPosition() + inchToEncoder( ( -2 * Math.PI * 13.625 ) * ( angle / 360 ) ));
        leftMotor.setPower( power );
        while (leftMotor.isBusy()) {
        }
        sleep( 200 );
    }

    public void turnOneRight( double angle, double power ) {
        rightMotor.setTargetPosition(rightMotor.getCurrentPosition() + inchToEncoder((-2 * Math.PI * 13.625) * (angle / 360)));
        rightMotor.setPower(power);
        while (rightMotor.isBusy()) {
        }
        sleep(200);
    }

    public void turnTwo( double angle, double power ) {
        leftMotor.setTargetPosition( leftMotor.getCurrentPosition() + inchToEncoder( ( Math.PI * 13.625 ) * ( angle / -360 ) ));
        rightMotor.setTargetPosition( rightMotor.getCurrentPosition() + inchToEncoder( ( Math.PI * 13.625 ) * ( angle / 360 ) ));
        leftMotor.setPower( power );
        rightMotor.setPower( power );
        while (leftMotor.isBusy() || rightMotor.isBusy()) {
        }
        sleep( 200 );
    }

}