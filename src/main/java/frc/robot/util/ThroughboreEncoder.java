package frc.robot.util;

import edu.wpi.first.wpilibj.DutyCycleEncoder;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.CounterBase.EncodingType;

public class ThroughboreEncoder {
    private final DutyCycleEncoder throughboreAbsoluteEncoder;
    private final Encoder throughboreRelativeEncoder;
    private final boolean reversed;

    public ThroughboreEncoder(int encoderAbsPort, int encoderAPort, int encoderBPort, double absoluteOffset, boolean reverse) {
        this.throughboreAbsoluteEncoder = new DutyCycleEncoder(encoderAbsPort);
        this.throughboreRelativeEncoder = new Encoder(encoderAPort, encoderBPort, false, EncodingType.k4X);

        this.throughboreAbsoluteEncoder.setPositionOffset(absoluteOffset);
        this.reversed = reverse;

        System.out.println(this.throughboreAbsoluteEncoder.isConnected());
    }

    public ThroughboreEncoder(int encoderAbsPort, int encoderAPort, int encoderBPort) {
        this(encoderAbsPort, encoderAPort, encoderBPort, 0, false);
    }

    public double getRelativeEncoderValue() {
        return this.throughboreRelativeEncoder.getDistance();
    }

    public double getAbsoluteEncoderValue() {
        return (reversed ? -1 : 1) * (this.throughboreAbsoluteEncoder.getAbsolutePosition() - this.throughboreAbsoluteEncoder.getPositionOffset()) * 360;
    }
}
