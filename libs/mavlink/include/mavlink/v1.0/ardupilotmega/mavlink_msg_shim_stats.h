// MESSAGE SHIM_STATS PACKING

#define MAVLINK_MSG_ID_SHIM_STATS 232

typedef struct __mavlink_shim_stats_t
{
 float percent_rejected; ///< The percentage of iterations in the last window_time milliseconds that the verified shim rejected the proposed acceleration and issued its own.
 float avg_accel_diff; ///< The average difference in the last window_time milliseconds between proposed acceleration and the acceleration issued by the verified shim.
 float x_position; ///< The value of x, relative to starting position.
 float height; ///< The height, relative to starting position.
 float vel_x; ///< The velocity in the x direction.
 float vel_vertical; ///< The velocity in the vertical direction.
 uint8_t set_motors_from_acc_failed; ///< 1 if set_motors_from_acc failed since the last time this message was sent. 0 otherwise.
} mavlink_shim_stats_t;

#define MAVLINK_MSG_ID_SHIM_STATS_LEN 25
#define MAVLINK_MSG_ID_232_LEN 25

#define MAVLINK_MSG_ID_SHIM_STATS_CRC 219
#define MAVLINK_MSG_ID_232_CRC 219



#define MAVLINK_MESSAGE_INFO_SHIM_STATS { \
	"SHIM_STATS", \
	7, \
	{  { "percent_rejected", NULL, MAVLINK_TYPE_FLOAT, 0, 0, offsetof(mavlink_shim_stats_t, percent_rejected) }, \
         { "avg_accel_diff", NULL, MAVLINK_TYPE_FLOAT, 0, 4, offsetof(mavlink_shim_stats_t, avg_accel_diff) }, \
         { "x_position", NULL, MAVLINK_TYPE_FLOAT, 0, 8, offsetof(mavlink_shim_stats_t, x_position) }, \
         { "height", NULL, MAVLINK_TYPE_FLOAT, 0, 12, offsetof(mavlink_shim_stats_t, height) }, \
         { "vel_x", NULL, MAVLINK_TYPE_FLOAT, 0, 16, offsetof(mavlink_shim_stats_t, vel_x) }, \
         { "vel_vertical", NULL, MAVLINK_TYPE_FLOAT, 0, 20, offsetof(mavlink_shim_stats_t, vel_vertical) }, \
         { "set_motors_from_acc_failed", NULL, MAVLINK_TYPE_UINT8_T, 0, 24, offsetof(mavlink_shim_stats_t, set_motors_from_acc_failed) }, \
         } \
}


/**
 * @brief Pack a shim_stats message
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param msg The MAVLink message to compress the data into
 *
 * @param percent_rejected The percentage of iterations in the last window_time milliseconds that the verified shim rejected the proposed acceleration and issued its own.
 * @param avg_accel_diff The average difference in the last window_time milliseconds between proposed acceleration and the acceleration issued by the verified shim.
 * @param set_motors_from_acc_failed 1 if set_motors_from_acc failed since the last time this message was sent. 0 otherwise.
 * @param x_position The value of x, relative to starting position.
 * @param height The height, relative to starting position.
 * @param vel_x The velocity in the x direction.
 * @param vel_vertical The velocity in the vertical direction.
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t mavlink_msg_shim_stats_pack(uint8_t system_id, uint8_t component_id, mavlink_message_t* msg,
						       float percent_rejected, float avg_accel_diff, uint8_t set_motors_from_acc_failed, float x_position, float height, float vel_x, float vel_vertical)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
	char buf[MAVLINK_MSG_ID_SHIM_STATS_LEN];
	_mav_put_float(buf, 0, percent_rejected);
	_mav_put_float(buf, 4, avg_accel_diff);
	_mav_put_float(buf, 8, x_position);
	_mav_put_float(buf, 12, height);
	_mav_put_float(buf, 16, vel_x);
	_mav_put_float(buf, 20, vel_vertical);
	_mav_put_uint8_t(buf, 24, set_motors_from_acc_failed);

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), buf, MAVLINK_MSG_ID_SHIM_STATS_LEN);
#else
	mavlink_shim_stats_t packet;
	packet.percent_rejected = percent_rejected;
	packet.avg_accel_diff = avg_accel_diff;
	packet.x_position = x_position;
	packet.height = height;
	packet.vel_x = vel_x;
	packet.vel_vertical = vel_vertical;
	packet.set_motors_from_acc_failed = set_motors_from_acc_failed;

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), &packet, MAVLINK_MSG_ID_SHIM_STATS_LEN);
#endif

	msg->msgid = MAVLINK_MSG_ID_SHIM_STATS;
#if MAVLINK_CRC_EXTRA
    return mavlink_finalize_message(msg, system_id, component_id, MAVLINK_MSG_ID_SHIM_STATS_LEN, MAVLINK_MSG_ID_SHIM_STATS_CRC);
#else
    return mavlink_finalize_message(msg, system_id, component_id, MAVLINK_MSG_ID_SHIM_STATS_LEN);
#endif
}

/**
 * @brief Pack a shim_stats message on a channel
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param chan The MAVLink channel this message will be sent over
 * @param msg The MAVLink message to compress the data into
 * @param percent_rejected The percentage of iterations in the last window_time milliseconds that the verified shim rejected the proposed acceleration and issued its own.
 * @param avg_accel_diff The average difference in the last window_time milliseconds between proposed acceleration and the acceleration issued by the verified shim.
 * @param set_motors_from_acc_failed 1 if set_motors_from_acc failed since the last time this message was sent. 0 otherwise.
 * @param x_position The value of x, relative to starting position.
 * @param height The height, relative to starting position.
 * @param vel_x The velocity in the x direction.
 * @param vel_vertical The velocity in the vertical direction.
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t mavlink_msg_shim_stats_pack_chan(uint8_t system_id, uint8_t component_id, uint8_t chan,
							   mavlink_message_t* msg,
						           float percent_rejected,float avg_accel_diff,uint8_t set_motors_from_acc_failed,float x_position,float height,float vel_x,float vel_vertical)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
	char buf[MAVLINK_MSG_ID_SHIM_STATS_LEN];
	_mav_put_float(buf, 0, percent_rejected);
	_mav_put_float(buf, 4, avg_accel_diff);
	_mav_put_float(buf, 8, x_position);
	_mav_put_float(buf, 12, height);
	_mav_put_float(buf, 16, vel_x);
	_mav_put_float(buf, 20, vel_vertical);
	_mav_put_uint8_t(buf, 24, set_motors_from_acc_failed);

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), buf, MAVLINK_MSG_ID_SHIM_STATS_LEN);
#else
	mavlink_shim_stats_t packet;
	packet.percent_rejected = percent_rejected;
	packet.avg_accel_diff = avg_accel_diff;
	packet.x_position = x_position;
	packet.height = height;
	packet.vel_x = vel_x;
	packet.vel_vertical = vel_vertical;
	packet.set_motors_from_acc_failed = set_motors_from_acc_failed;

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), &packet, MAVLINK_MSG_ID_SHIM_STATS_LEN);
#endif

	msg->msgid = MAVLINK_MSG_ID_SHIM_STATS;
#if MAVLINK_CRC_EXTRA
    return mavlink_finalize_message_chan(msg, system_id, component_id, chan, MAVLINK_MSG_ID_SHIM_STATS_LEN, MAVLINK_MSG_ID_SHIM_STATS_CRC);
#else
    return mavlink_finalize_message_chan(msg, system_id, component_id, chan, MAVLINK_MSG_ID_SHIM_STATS_LEN);
#endif
}

/**
 * @brief Encode a shim_stats struct
 *
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param msg The MAVLink message to compress the data into
 * @param shim_stats C-struct to read the message contents from
 */
static inline uint16_t mavlink_msg_shim_stats_encode(uint8_t system_id, uint8_t component_id, mavlink_message_t* msg, const mavlink_shim_stats_t* shim_stats)
{
	return mavlink_msg_shim_stats_pack(system_id, component_id, msg, shim_stats->percent_rejected, shim_stats->avg_accel_diff, shim_stats->set_motors_from_acc_failed, shim_stats->x_position, shim_stats->height, shim_stats->vel_x, shim_stats->vel_vertical);
}

/**
 * @brief Encode a shim_stats struct on a channel
 *
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param chan The MAVLink channel this message will be sent over
 * @param msg The MAVLink message to compress the data into
 * @param shim_stats C-struct to read the message contents from
 */
static inline uint16_t mavlink_msg_shim_stats_encode_chan(uint8_t system_id, uint8_t component_id, uint8_t chan, mavlink_message_t* msg, const mavlink_shim_stats_t* shim_stats)
{
	return mavlink_msg_shim_stats_pack_chan(system_id, component_id, chan, msg, shim_stats->percent_rejected, shim_stats->avg_accel_diff, shim_stats->set_motors_from_acc_failed, shim_stats->x_position, shim_stats->height, shim_stats->vel_x, shim_stats->vel_vertical);
}

/**
 * @brief Send a shim_stats message
 * @param chan MAVLink channel to send the message
 *
 * @param percent_rejected The percentage of iterations in the last window_time milliseconds that the verified shim rejected the proposed acceleration and issued its own.
 * @param avg_accel_diff The average difference in the last window_time milliseconds between proposed acceleration and the acceleration issued by the verified shim.
 * @param set_motors_from_acc_failed 1 if set_motors_from_acc failed since the last time this message was sent. 0 otherwise.
 * @param x_position The value of x, relative to starting position.
 * @param height The height, relative to starting position.
 * @param vel_x The velocity in the x direction.
 * @param vel_vertical The velocity in the vertical direction.
 */
#ifdef MAVLINK_USE_CONVENIENCE_FUNCTIONS

static inline void mavlink_msg_shim_stats_send(mavlink_channel_t chan, float percent_rejected, float avg_accel_diff, uint8_t set_motors_from_acc_failed, float x_position, float height, float vel_x, float vel_vertical)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
	char buf[MAVLINK_MSG_ID_SHIM_STATS_LEN];
	_mav_put_float(buf, 0, percent_rejected);
	_mav_put_float(buf, 4, avg_accel_diff);
	_mav_put_float(buf, 8, x_position);
	_mav_put_float(buf, 12, height);
	_mav_put_float(buf, 16, vel_x);
	_mav_put_float(buf, 20, vel_vertical);
	_mav_put_uint8_t(buf, 24, set_motors_from_acc_failed);

#if MAVLINK_CRC_EXTRA
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_SHIM_STATS, buf, MAVLINK_MSG_ID_SHIM_STATS_LEN, MAVLINK_MSG_ID_SHIM_STATS_CRC);
#else
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_SHIM_STATS, buf, MAVLINK_MSG_ID_SHIM_STATS_LEN);
#endif
#else
	mavlink_shim_stats_t packet;
	packet.percent_rejected = percent_rejected;
	packet.avg_accel_diff = avg_accel_diff;
	packet.x_position = x_position;
	packet.height = height;
	packet.vel_x = vel_x;
	packet.vel_vertical = vel_vertical;
	packet.set_motors_from_acc_failed = set_motors_from_acc_failed;

#if MAVLINK_CRC_EXTRA
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_SHIM_STATS, (const char *)&packet, MAVLINK_MSG_ID_SHIM_STATS_LEN, MAVLINK_MSG_ID_SHIM_STATS_CRC);
#else
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_SHIM_STATS, (const char *)&packet, MAVLINK_MSG_ID_SHIM_STATS_LEN);
#endif
#endif
}

#if MAVLINK_MSG_ID_SHIM_STATS_LEN <= MAVLINK_MAX_PAYLOAD_LEN
/*
  This varient of _send() can be used to save stack space by re-using
  memory from the receive buffer.  The caller provides a
  mavlink_message_t which is the size of a full mavlink message. This
  is usually the receive buffer for the channel, and allows a reply to an
  incoming message with minimum stack space usage.
 */
static inline void mavlink_msg_shim_stats_send_buf(mavlink_message_t *msgbuf, mavlink_channel_t chan,  float percent_rejected, float avg_accel_diff, uint8_t set_motors_from_acc_failed, float x_position, float height, float vel_x, float vel_vertical)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
	char *buf = (char *)msgbuf;
	_mav_put_float(buf, 0, percent_rejected);
	_mav_put_float(buf, 4, avg_accel_diff);
	_mav_put_float(buf, 8, x_position);
	_mav_put_float(buf, 12, height);
	_mav_put_float(buf, 16, vel_x);
	_mav_put_float(buf, 20, vel_vertical);
	_mav_put_uint8_t(buf, 24, set_motors_from_acc_failed);

#if MAVLINK_CRC_EXTRA
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_SHIM_STATS, buf, MAVLINK_MSG_ID_SHIM_STATS_LEN, MAVLINK_MSG_ID_SHIM_STATS_CRC);
#else
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_SHIM_STATS, buf, MAVLINK_MSG_ID_SHIM_STATS_LEN);
#endif
#else
	mavlink_shim_stats_t *packet = (mavlink_shim_stats_t *)msgbuf;
	packet->percent_rejected = percent_rejected;
	packet->avg_accel_diff = avg_accel_diff;
	packet->x_position = x_position;
	packet->height = height;
	packet->vel_x = vel_x;
	packet->vel_vertical = vel_vertical;
	packet->set_motors_from_acc_failed = set_motors_from_acc_failed;

#if MAVLINK_CRC_EXTRA
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_SHIM_STATS, (const char *)packet, MAVLINK_MSG_ID_SHIM_STATS_LEN, MAVLINK_MSG_ID_SHIM_STATS_CRC);
#else
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_SHIM_STATS, (const char *)packet, MAVLINK_MSG_ID_SHIM_STATS_LEN);
#endif
#endif
}
#endif

#endif

// MESSAGE SHIM_STATS UNPACKING


/**
 * @brief Get field percent_rejected from shim_stats message
 *
 * @return The percentage of iterations in the last window_time milliseconds that the verified shim rejected the proposed acceleration and issued its own.
 */
static inline float mavlink_msg_shim_stats_get_percent_rejected(const mavlink_message_t* msg)
{
	return _MAV_RETURN_float(msg,  0);
}

/**
 * @brief Get field avg_accel_diff from shim_stats message
 *
 * @return The average difference in the last window_time milliseconds between proposed acceleration and the acceleration issued by the verified shim.
 */
static inline float mavlink_msg_shim_stats_get_avg_accel_diff(const mavlink_message_t* msg)
{
	return _MAV_RETURN_float(msg,  4);
}

/**
 * @brief Get field set_motors_from_acc_failed from shim_stats message
 *
 * @return 1 if set_motors_from_acc failed since the last time this message was sent. 0 otherwise.
 */
static inline uint8_t mavlink_msg_shim_stats_get_set_motors_from_acc_failed(const mavlink_message_t* msg)
{
	return _MAV_RETURN_uint8_t(msg,  24);
}

/**
 * @brief Get field x_position from shim_stats message
 *
 * @return The value of x, relative to starting position.
 */
static inline float mavlink_msg_shim_stats_get_x_position(const mavlink_message_t* msg)
{
	return _MAV_RETURN_float(msg,  8);
}

/**
 * @brief Get field height from shim_stats message
 *
 * @return The height, relative to starting position.
 */
static inline float mavlink_msg_shim_stats_get_height(const mavlink_message_t* msg)
{
	return _MAV_RETURN_float(msg,  12);
}

/**
 * @brief Get field vel_x from shim_stats message
 *
 * @return The velocity in the x direction.
 */
static inline float mavlink_msg_shim_stats_get_vel_x(const mavlink_message_t* msg)
{
	return _MAV_RETURN_float(msg,  16);
}

/**
 * @brief Get field vel_vertical from shim_stats message
 *
 * @return The velocity in the vertical direction.
 */
static inline float mavlink_msg_shim_stats_get_vel_vertical(const mavlink_message_t* msg)
{
	return _MAV_RETURN_float(msg,  20);
}

/**
 * @brief Decode a shim_stats message into a struct
 *
 * @param msg The message to decode
 * @param shim_stats C-struct to decode the message contents into
 */
static inline void mavlink_msg_shim_stats_decode(const mavlink_message_t* msg, mavlink_shim_stats_t* shim_stats)
{
#if MAVLINK_NEED_BYTE_SWAP
	shim_stats->percent_rejected = mavlink_msg_shim_stats_get_percent_rejected(msg);
	shim_stats->avg_accel_diff = mavlink_msg_shim_stats_get_avg_accel_diff(msg);
	shim_stats->x_position = mavlink_msg_shim_stats_get_x_position(msg);
	shim_stats->height = mavlink_msg_shim_stats_get_height(msg);
	shim_stats->vel_x = mavlink_msg_shim_stats_get_vel_x(msg);
	shim_stats->vel_vertical = mavlink_msg_shim_stats_get_vel_vertical(msg);
	shim_stats->set_motors_from_acc_failed = mavlink_msg_shim_stats_get_set_motors_from_acc_failed(msg);
#else
	memcpy(shim_stats, _MAV_PAYLOAD(msg), MAVLINK_MSG_ID_SHIM_STATS_LEN);
#endif
}
