// MESSAGE THROTTLE_PWM_STATS PACKING

#define MAVLINK_MSG_ID_THROTTLE_PWM_STATS 233

typedef struct __mavlink_throttle_pwm_stats_t
{
 uint32_t window_time; ///< The time window in milliseconds over which the averages in this message should be taken
 float pwm_avg; ///< The average pwm of a motor in the last window_time milliseconds
 float throttle_avg; ///< The average throttle passed to the motor mixing code in the last window_time milliseconds
} mavlink_throttle_pwm_stats_t;

#define MAVLINK_MSG_ID_THROTTLE_PWM_STATS_LEN 12
#define MAVLINK_MSG_ID_233_LEN 12

#define MAVLINK_MSG_ID_THROTTLE_PWM_STATS_CRC 102
#define MAVLINK_MSG_ID_233_CRC 102



#define MAVLINK_MESSAGE_INFO_THROTTLE_PWM_STATS { \
	"THROTTLE_PWM_STATS", \
	3, \
	{  { "window_time", NULL, MAVLINK_TYPE_UINT32_T, 0, 0, offsetof(mavlink_throttle_pwm_stats_t, window_time) }, \
         { "pwm_avg", NULL, MAVLINK_TYPE_FLOAT, 0, 4, offsetof(mavlink_throttle_pwm_stats_t, pwm_avg) }, \
         { "throttle_avg", NULL, MAVLINK_TYPE_FLOAT, 0, 8, offsetof(mavlink_throttle_pwm_stats_t, throttle_avg) }, \
         } \
}


/**
 * @brief Pack a throttle_pwm_stats message
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param msg The MAVLink message to compress the data into
 *
 * @param window_time The time window in milliseconds over which the averages in this message should be taken
 * @param pwm_avg The average pwm of a motor in the last window_time milliseconds
 * @param throttle_avg The average throttle passed to the motor mixing code in the last window_time milliseconds
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t mavlink_msg_throttle_pwm_stats_pack(uint8_t system_id, uint8_t component_id, mavlink_message_t* msg,
						       uint32_t window_time, float pwm_avg, float throttle_avg)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
	char buf[MAVLINK_MSG_ID_THROTTLE_PWM_STATS_LEN];
	_mav_put_uint32_t(buf, 0, window_time);
	_mav_put_float(buf, 4, pwm_avg);
	_mav_put_float(buf, 8, throttle_avg);

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), buf, MAVLINK_MSG_ID_THROTTLE_PWM_STATS_LEN);
#else
	mavlink_throttle_pwm_stats_t packet;
	packet.window_time = window_time;
	packet.pwm_avg = pwm_avg;
	packet.throttle_avg = throttle_avg;

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), &packet, MAVLINK_MSG_ID_THROTTLE_PWM_STATS_LEN);
#endif

	msg->msgid = MAVLINK_MSG_ID_THROTTLE_PWM_STATS;
#if MAVLINK_CRC_EXTRA
    return mavlink_finalize_message(msg, system_id, component_id, MAVLINK_MSG_ID_THROTTLE_PWM_STATS_LEN, MAVLINK_MSG_ID_THROTTLE_PWM_STATS_CRC);
#else
    return mavlink_finalize_message(msg, system_id, component_id, MAVLINK_MSG_ID_THROTTLE_PWM_STATS_LEN);
#endif
}

/**
 * @brief Pack a throttle_pwm_stats message on a channel
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param chan The MAVLink channel this message will be sent over
 * @param msg The MAVLink message to compress the data into
 * @param window_time The time window in milliseconds over which the averages in this message should be taken
 * @param pwm_avg The average pwm of a motor in the last window_time milliseconds
 * @param throttle_avg The average throttle passed to the motor mixing code in the last window_time milliseconds
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t mavlink_msg_throttle_pwm_stats_pack_chan(uint8_t system_id, uint8_t component_id, uint8_t chan,
							   mavlink_message_t* msg,
						           uint32_t window_time,float pwm_avg,float throttle_avg)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
	char buf[MAVLINK_MSG_ID_THROTTLE_PWM_STATS_LEN];
	_mav_put_uint32_t(buf, 0, window_time);
	_mav_put_float(buf, 4, pwm_avg);
	_mav_put_float(buf, 8, throttle_avg);

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), buf, MAVLINK_MSG_ID_THROTTLE_PWM_STATS_LEN);
#else
	mavlink_throttle_pwm_stats_t packet;
	packet.window_time = window_time;
	packet.pwm_avg = pwm_avg;
	packet.throttle_avg = throttle_avg;

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), &packet, MAVLINK_MSG_ID_THROTTLE_PWM_STATS_LEN);
#endif

	msg->msgid = MAVLINK_MSG_ID_THROTTLE_PWM_STATS;
#if MAVLINK_CRC_EXTRA
    return mavlink_finalize_message_chan(msg, system_id, component_id, chan, MAVLINK_MSG_ID_THROTTLE_PWM_STATS_LEN, MAVLINK_MSG_ID_THROTTLE_PWM_STATS_CRC);
#else
    return mavlink_finalize_message_chan(msg, system_id, component_id, chan, MAVLINK_MSG_ID_THROTTLE_PWM_STATS_LEN);
#endif
}

/**
 * @brief Encode a throttle_pwm_stats struct
 *
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param msg The MAVLink message to compress the data into
 * @param throttle_pwm_stats C-struct to read the message contents from
 */
static inline uint16_t mavlink_msg_throttle_pwm_stats_encode(uint8_t system_id, uint8_t component_id, mavlink_message_t* msg, const mavlink_throttle_pwm_stats_t* throttle_pwm_stats)
{
	return mavlink_msg_throttle_pwm_stats_pack(system_id, component_id, msg, throttle_pwm_stats->window_time, throttle_pwm_stats->pwm_avg, throttle_pwm_stats->throttle_avg);
}

/**
 * @brief Encode a throttle_pwm_stats struct on a channel
 *
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param chan The MAVLink channel this message will be sent over
 * @param msg The MAVLink message to compress the data into
 * @param throttle_pwm_stats C-struct to read the message contents from
 */
static inline uint16_t mavlink_msg_throttle_pwm_stats_encode_chan(uint8_t system_id, uint8_t component_id, uint8_t chan, mavlink_message_t* msg, const mavlink_throttle_pwm_stats_t* throttle_pwm_stats)
{
	return mavlink_msg_throttle_pwm_stats_pack_chan(system_id, component_id, chan, msg, throttle_pwm_stats->window_time, throttle_pwm_stats->pwm_avg, throttle_pwm_stats->throttle_avg);
}

/**
 * @brief Send a throttle_pwm_stats message
 * @param chan MAVLink channel to send the message
 *
 * @param window_time The time window in milliseconds over which the averages in this message should be taken
 * @param pwm_avg The average pwm of a motor in the last window_time milliseconds
 * @param throttle_avg The average throttle passed to the motor mixing code in the last window_time milliseconds
 */
#ifdef MAVLINK_USE_CONVENIENCE_FUNCTIONS

static inline void mavlink_msg_throttle_pwm_stats_send(mavlink_channel_t chan, uint32_t window_time, float pwm_avg, float throttle_avg)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
	char buf[MAVLINK_MSG_ID_THROTTLE_PWM_STATS_LEN];
	_mav_put_uint32_t(buf, 0, window_time);
	_mav_put_float(buf, 4, pwm_avg);
	_mav_put_float(buf, 8, throttle_avg);

#if MAVLINK_CRC_EXTRA
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_THROTTLE_PWM_STATS, buf, MAVLINK_MSG_ID_THROTTLE_PWM_STATS_LEN, MAVLINK_MSG_ID_THROTTLE_PWM_STATS_CRC);
#else
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_THROTTLE_PWM_STATS, buf, MAVLINK_MSG_ID_THROTTLE_PWM_STATS_LEN);
#endif
#else
	mavlink_throttle_pwm_stats_t packet;
	packet.window_time = window_time;
	packet.pwm_avg = pwm_avg;
	packet.throttle_avg = throttle_avg;

#if MAVLINK_CRC_EXTRA
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_THROTTLE_PWM_STATS, (const char *)&packet, MAVLINK_MSG_ID_THROTTLE_PWM_STATS_LEN, MAVLINK_MSG_ID_THROTTLE_PWM_STATS_CRC);
#else
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_THROTTLE_PWM_STATS, (const char *)&packet, MAVLINK_MSG_ID_THROTTLE_PWM_STATS_LEN);
#endif
#endif
}

#if MAVLINK_MSG_ID_THROTTLE_PWM_STATS_LEN <= MAVLINK_MAX_PAYLOAD_LEN
/*
  This varient of _send() can be used to save stack space by re-using
  memory from the receive buffer.  The caller provides a
  mavlink_message_t which is the size of a full mavlink message. This
  is usually the receive buffer for the channel, and allows a reply to an
  incoming message with minimum stack space usage.
 */
static inline void mavlink_msg_throttle_pwm_stats_send_buf(mavlink_message_t *msgbuf, mavlink_channel_t chan,  uint32_t window_time, float pwm_avg, float throttle_avg)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
	char *buf = (char *)msgbuf;
	_mav_put_uint32_t(buf, 0, window_time);
	_mav_put_float(buf, 4, pwm_avg);
	_mav_put_float(buf, 8, throttle_avg);

#if MAVLINK_CRC_EXTRA
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_THROTTLE_PWM_STATS, buf, MAVLINK_MSG_ID_THROTTLE_PWM_STATS_LEN, MAVLINK_MSG_ID_THROTTLE_PWM_STATS_CRC);
#else
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_THROTTLE_PWM_STATS, buf, MAVLINK_MSG_ID_THROTTLE_PWM_STATS_LEN);
#endif
#else
	mavlink_throttle_pwm_stats_t *packet = (mavlink_throttle_pwm_stats_t *)msgbuf;
	packet->window_time = window_time;
	packet->pwm_avg = pwm_avg;
	packet->throttle_avg = throttle_avg;

#if MAVLINK_CRC_EXTRA
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_THROTTLE_PWM_STATS, (const char *)packet, MAVLINK_MSG_ID_THROTTLE_PWM_STATS_LEN, MAVLINK_MSG_ID_THROTTLE_PWM_STATS_CRC);
#else
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_THROTTLE_PWM_STATS, (const char *)packet, MAVLINK_MSG_ID_THROTTLE_PWM_STATS_LEN);
#endif
#endif
}
#endif

#endif

// MESSAGE THROTTLE_PWM_STATS UNPACKING


/**
 * @brief Get field window_time from throttle_pwm_stats message
 *
 * @return The time window in milliseconds over which the averages in this message should be taken
 */
static inline uint32_t mavlink_msg_throttle_pwm_stats_get_window_time(const mavlink_message_t* msg)
{
	return _MAV_RETURN_uint32_t(msg,  0);
}

/**
 * @brief Get field pwm_avg from throttle_pwm_stats message
 *
 * @return The average pwm of a motor in the last window_time milliseconds
 */
static inline float mavlink_msg_throttle_pwm_stats_get_pwm_avg(const mavlink_message_t* msg)
{
	return _MAV_RETURN_float(msg,  4);
}

/**
 * @brief Get field throttle_avg from throttle_pwm_stats message
 *
 * @return The average throttle passed to the motor mixing code in the last window_time milliseconds
 */
static inline float mavlink_msg_throttle_pwm_stats_get_throttle_avg(const mavlink_message_t* msg)
{
	return _MAV_RETURN_float(msg,  8);
}

/**
 * @brief Decode a throttle_pwm_stats message into a struct
 *
 * @param msg The message to decode
 * @param throttle_pwm_stats C-struct to decode the message contents into
 */
static inline void mavlink_msg_throttle_pwm_stats_decode(const mavlink_message_t* msg, mavlink_throttle_pwm_stats_t* throttle_pwm_stats)
{
#if MAVLINK_NEED_BYTE_SWAP
	throttle_pwm_stats->window_time = mavlink_msg_throttle_pwm_stats_get_window_time(msg);
	throttle_pwm_stats->pwm_avg = mavlink_msg_throttle_pwm_stats_get_pwm_avg(msg);
	throttle_pwm_stats->throttle_avg = mavlink_msg_throttle_pwm_stats_get_throttle_avg(msg);
#else
	memcpy(throttle_pwm_stats, _MAV_PAYLOAD(msg), MAVLINK_MSG_ID_THROTTLE_PWM_STATS_LEN);
#endif
}
