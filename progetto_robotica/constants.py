"""Shared ROS topic names and recording profiles."""

TOPIC_CMD_VEL = "/cmd_vel"
TOPIC_IMU = "/imu"
TOPIC_JOINT_STATES = "/joint_states"
TOPIC_CONTACT_LEFT = "/contacts/left"
TOPIC_CONTACT_RIGHT = "/contacts/right"
TOPIC_ODOM = "/odom"
TOPIC_FALL_DETECTED = "/fall_detected"
TOPIC_CMD_LATENCY_MS = "/metrics/cmd_latency_ms"
TOPIC_RECORDING_STATUS = "/recording_status"
TOPIC_SIM_RESET = "/sim_reset"

RECORDING_TOPIC_PROFILES = {
    "minimal": [
        TOPIC_CMD_VEL,
        TOPIC_CONTACT_LEFT,
        TOPIC_CONTACT_RIGHT,
        TOPIC_FALL_DETECTED,
        TOPIC_CMD_LATENCY_MS,
    ],
    "metrics": [
        TOPIC_CMD_VEL,
        TOPIC_CONTACT_LEFT,
        TOPIC_CONTACT_RIGHT,
        TOPIC_ODOM,
        TOPIC_FALL_DETECTED,
        TOPIC_CMD_LATENCY_MS,
    ],
    "full": [
        TOPIC_CMD_VEL,
        TOPIC_IMU,
        TOPIC_JOINT_STATES,
        TOPIC_CONTACT_LEFT,
        TOPIC_CONTACT_RIGHT,
        TOPIC_ODOM,
        TOPIC_FALL_DETECTED,
        TOPIC_CMD_LATENCY_MS,
    ],
}
