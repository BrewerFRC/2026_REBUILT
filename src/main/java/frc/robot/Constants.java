package frc.robot;

import com.ctre.phoenix6.CANBus;

/**
 * Robot-wide configuration values.
 *
 * <p>Keep hardware IDs and operator-interface ports here instead of scattering numeric literals
 * throughout subsystems. Values are {@code static final}: code may read them but must not change
 * the robot's wiring configuration while it is running.
 */
public final class Constants {
  private Constants() {}

  /** Hardware and controller assignments. Update these to match the electrical layout. */
  public static final class OperatorConstants {
    private OperatorConstants() {}

    /** PWM channel for the hopper's brushed Spark controller. */
    public static final int HOPPER_MOTOR_PWM_CHANNEL = 3;

    /** Named CANivore bus used by all CTRE motors on this robot. */
    public static final CANBus CANIVORE_BUS = new CANBus("4564_Canivore_01");

    public static final int DRIVER_CONTROLLER_PORT = 0;
    public static final int OPERATOR_CONTROLLER_PORT = 1;

    public static final int ARM_MOTOR_CAN_ID = 51;
    public static final int INTAKE_MOTOR_CAN_ID = 52;
    public static final int FEEDER_MOTOR_CAN_ID = 53;
    public static final int SHOOTER_LEFT_MOTOR_CAN_ID = 56;
    public static final int SHOOTER_RIGHT_MOTOR_CAN_ID = 57;
    public static final int HOOD_MOTOR_CAN_ID = 59;
  }
}
