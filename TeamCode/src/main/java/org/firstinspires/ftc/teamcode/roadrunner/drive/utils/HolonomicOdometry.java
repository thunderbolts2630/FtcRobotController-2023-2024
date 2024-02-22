package org.firstinspires.ftc.teamcode.roadrunner.drive.utils;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.roadrunner.drive.utils.geometry.BTPose2d;
import org.firstinspires.ftc.teamcode.roadrunner.drive.utils.geometry.BTRotation2d;
import org.firstinspires.ftc.teamcode.roadrunner.drive.utils.geometry.BTTwist2d;
import org.firstinspires.ftc.teamcode.utils.geometry.*;

import java.util.function.DoubleSupplier;

public class HolonomicOdometry   {

    private double prevLeftEncoder, prevRightEncoder, prevHorizontalEncoder;
    private BTRotation2d previousAngle;
    private double centerWheelOffset;
    protected BTPose2d robotPose = new BTPose2d();
    private double dx;  // not null
    private double dt;// not null
    private double dv;
    private ElapsedTime time;
    private double previousTime;
    private double prevVelocity = 0;
    boolean isFirstTime = true;

    public BTPose2d getPose() {
        return robotPose;
    }


    /**
     * The trackwidth of the odometers
     */
    protected double m_trackWidth;

    // the suppliers
    DoubleSupplier m_left, m_right, m_horizontal, m_gyro;

    public HolonomicOdometry(DoubleSupplier leftEncoder, DoubleSupplier rightEncoder,
                             DoubleSupplier horizontalEncoder, DoubleSupplier gyroAngle, double trackWidth, double centerWheelOffset) {
        this(trackWidth, centerWheelOffset);
        m_left = leftEncoder;
        m_right = rightEncoder;
        m_horizontal = horizontalEncoder;
        m_gyro = gyroAngle;

    }

    public HolonomicOdometry(BTPose2d initialPose, double trackwidth, double centerWheelOffset) {
        this.m_trackWidth = trackwidth;
        previousAngle = initialPose.getRotation();
        robotPose=initialPose;
        this.centerWheelOffset = centerWheelOffset;
        time = new ElapsedTime(ElapsedTime.Resolution.SECONDS);
        previousTime = time.time();
    }

    public HolonomicOdometry(double trackwidth, double centerWheelOffset) {
        this(new BTPose2d(), trackwidth, centerWheelOffset);
    }

    /**
     * This handles all the calculations for you.
     */
    public void updatePose() {
        update(m_left.getAsDouble(), m_right.getAsDouble(), m_horizontal.getAsDouble(), m_gyro.getAsDouble());
    }

    public void setPose(BTPose2d pose) {
        previousAngle = pose.getRotation();
        robotPose = pose;

        prevLeftEncoder = 0;
        prevRightEncoder = 0;
        prevHorizontalEncoder = 0;
    }

    public void update(double leftEncoderPos, double rightEncoderPos, double horizontalEncoderPos,double gyroAngle) {
        double deltaLeftEncoder = leftEncoderPos - prevLeftEncoder;
        double deltaRightEncoder = rightEncoderPos - prevRightEncoder;
        double deltaHorizontalEncoder = horizontalEncoderPos - prevHorizontalEncoder;

        BTRotation2d angle = BTRotation2d.fromDegrees(gyroAngle);

        prevLeftEncoder = leftEncoderPos;
        prevRightEncoder = rightEncoderPos;
        prevHorizontalEncoder = horizontalEncoderPos;

        dt = time.time() - previousTime;
        double dw = (angle.minus(previousAngle).getRadians());
        double dy = deltaHorizontalEncoder - (centerWheelOffset * dw);
        dx = (deltaLeftEncoder + deltaRightEncoder) / 2;

        dv = getVelocity() - prevVelocity;
        prevVelocity = getVelocity();
        BTTwist2d twist2d = new BTTwist2d(dx, dy, dw);

        BTPose2d newPose = robotPose.exp(twist2d);

        previousAngle = angle;
        previousTime = time.time();
        robotPose = new BTPose2d(newPose.getTranslation(), angle);
        isFirstTime = false;
    }

    public void reset(BTPose2d btPose2d) {
        previousAngle=BTRotation2d.fromDegrees(0);
        prevLeftEncoder=0;
        prevRightEncoder=0;
        prevHorizontalEncoder=0;
        robotPose=btPose2d;
    }

    public double getVelocity(){
        double velocity;
        if (!isFirstTime){
            velocity = 0;
        }else{
            velocity = dx/dt;
        }
        return velocity;
    }
    public double getAcceleration(){
        double acceleration;
        if(!isFirstTime){
            acceleration = 0;
        }else{
            acceleration = dv/dt;
        }
        return acceleration;
    }


}
