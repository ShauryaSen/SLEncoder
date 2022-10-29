import java.nio.ByteOrder;
import java.nio.ByteBuffer;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.PeriodicFrame;

import edu.wpi.first.hal.CANData;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.filter.LinearFilter;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.CAN;
import edu.wpi.first.wpilibj.Notifier;


public class SLEncoder {

    private static final int deviceManufacturer = 5; // REV
    private static final int deviceType = 2; // Spark Max
    private static final int apiId = 98; // Periodic status 2
  
    private final CANSparkMax sparkMax;
    private final CAN canInterface;
    private final LinearFilter velocityFilter;
    private final Notifier notifier;
  
    private boolean firstCycle = true;
    private boolean enabled = false;
    private double ffVolts = 0.0;
    private double timestamp = 0.0;
    private double position = 0.0;
    private double velocity = 0.0;

    /**
   * Creates a new SLEncoder using a default set of parameters.
   */
  public SLEncoder(CANSparkMax sparkMax) {
    this(sparkMax, 0.02, 5);
  }

    /* Creates a new SLEncoder. */
    public SLEncoder(CANSparkMax sparkMax, double periodSeconds, int averagingTaps) {
        this.sparkMax = sparkMax;
        sparkMax.getEncoder().setPositionConversionFactor(1.0);
        int periodMs = (int) (periodSeconds * 1000);
        sparkMax.setPeriodicFramePeriod(PeriodicFrame.kStatus2, periodMs);

        canInterface =
            new CAN(sparkMax.getDeviceId(), deviceManufacturer, deviceType);
        velocityFilter = LinearFilter.movingAverage(averagingTaps);

        notifier = new Notifier(this::update);
        notifier.startPeriodic(periodSeconds);
    }

    private void update() {
        CANData canData = new CANData();
        boolean isFresh = canInterface.readPacketNew(apiId, canData);
        double newTimestamp = canData.timestamp / 1000.0;
        double newPosition = ByteBuffer.wrap(canData.data)
        .order(ByteOrder.LITTLE_ENDIAN).asFloatBuffer().get(0);

        if (isFresh) {
            synchronized (this) {
                if (!firstCycle) {
                    velocity = velocityFilter.calculate( 
                        (newPosition - position) / 
                        (newTimestamp - timestamp) * 60
                    );
                } else { firstCycle = false; }

                // update the values
                timestamp = newTimestamp;
                position = newPosition;
            }
        }
    }


    /* Returns the current position in rotations */
    public synchronized double getPosition() {
        return position;
    }

    /* Returns the current velocity in rotations/minute. */
    public synchronized double getVelocity() {
        return velocity;
    }
    
}