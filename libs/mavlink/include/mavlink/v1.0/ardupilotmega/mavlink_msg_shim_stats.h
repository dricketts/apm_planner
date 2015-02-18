// MESSAGE SHIM_STATS PACKING

#define MAVLINK_MSG_ID_SHIM_STATS 232

typedef struct __mavlink_shim_stats_t
{
 uint64_t time; ///< The time of these statistics in milliseconds since the epoch.
 float percent_rejected_ver; ///< The percentage of iterations in the last window_time milliseconds that the verified shim rejected the proposed acceleration and issued its own.
 float percent_rejected_unver; ///< The percentage of iterations in the last window_time milliseconds that the unverified smoothing shim rejected the proposed acceleration and issued its own.
 float avg_accel_diff_ver; ///< The average difference in the last window_time milliseconds between proposed acceleration and the acceleration issued by the verified shim.
 float avg_accel_diff_unver; ///< The average difference in the last window_time milliseconds between proposed acceleration and the acceleration issued by the unverified shim.
} mavlink_shim_stats_t;

#define MAVLINK_MSG_ID_SHIM_STATS_LEN 24
#define MAVLINK_MSG_ID_232_LEN 24

#define MAVLINK_MSG_ID_SHIM_STATS_CRC 169
#define MAVLINK_MSG_ID_232_CRC 169



#define MAVLINK_MESSAGE_INFO_SHIM_STATS { \
	"SHIM_STATS", \
	5, \
	{  { "time", NULL, MAVLINK_TYPE_UINT64_T, 0, 0, offsetof(mavlink_shim_stats_t, time) }, \
         { "percent_rejected_ver", NULL, MAVLINK_TYPE_FLOAT, 0, 8, offsetof(mavlink_shim_stats_t, percent_rejected_ver) }, \
         { "percent_rejected_unver", NULL, MAVLINK_TYPE_FLOAT, 0, 12, offsetof(mavlink_shim_stats_t, percent_rejected_unver) }, \
         { "avg_accel_diff_ver", NULL, MAVLINK_TYPE_FLOAT, 0, 16, offsetof(mavlink_shim_stats_t, avg_accel_diff_ver) }, \
         { "avg_accel_diff_unver", NULL, MAVLINK_TYPE_FLOAT, 0, 20, offsetof(mavlink_shim_stats_t, avg_accel_diff_unver) }, \
         } \
}


/**
 * @brief Pack a shim_stats message
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param msg The MAVLink message to compress the data into
 *
 * @param time The time of these statistics in milliseconds since the epoch.
 * @param percent_rejected_ver The percentage of iterations in the last window_time milliseconds that the verified shim rejected the proposed acceleration and issued its own.
 * @param percent_rejected_unver The percentage of iterations in the last window_time milliseconds that the unverified smoothing shim rejected the proposed acceleration and issued its own.
 * @param avg_accel_diff_ver The average difference in the last window_time milliseconds between proposed acceleration and the acceleration issued by the verified shim.
 * @param avg_accel_diff_unver The average difference in the last window_time milliseconds between proposed acceleration and the acceleration issued by the unverified shim.
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t mavlink_msg_shim_stats_pack(uint8_t system_id, uint8_t component_id, mavlink_message_t* msg,
						       uint64_t time, float percent_rejected_ver, float percent_rejected_unver, float avg_accel_diff_ver, float avg_accel_diff_unver)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
	char buf[MAVLINK_MSG_ID_SHIM_STATS_LEN];
	_mav_put_uint64_t(buf, 0, time);
	_mav_put_float(buf, 8, percent_rejected_ver);
	_mav_put_float(buf, 12, percent_rejected_unver);
	_mav_put_float(buf, 16, avg_accel_diff_ver);
	_mav_put_float(buf, 20, avg_accel_diff_unver);

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), buf, MAVLINK_MSG_ID_SHIM_STATS_LEN);
#else
	mavlink_shim_stats_t packet;
	packet.time = time;
	packet.percent_rejected_ver = percent_rejected_ver;
	packet.percent_rejected_unver = percent_rejected_unver;
	packet.avg_accel_diff_ver = avg_accel_diff_ver;
	packet.avg_accel_diff_unver = avg_accel_diff_unver;

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
 * @param time The time of these statistics in milliseconds since the epoch.
 * @param percent_rejected_ver The percentage of iterations in the last window_time milliseconds that the verified shim rejected the proposed acceleration and issued its own.
 * @param percent_rejected_unver The percentage of iterations in the last window_time milliseconds that the unverified smoothing shim rejected the proposed acceleration and issued its own.
 * @param avg_accel_diff_ver The average difference in the last window_time milliseconds between proposed acceleration and the acceleration issued by the verified shim.
 * @param avg_accel_diff_unver The average difference in the last window_time milliseconds between proposed acceleration and the acceleration issued by the unverified shim.
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t mavlink_msg_shim_stats_pack_chan(uint8_t system_id, uint8_t component_id, uint8_t chan,
							   mavlink_message_t* msg,
						           uint64_t time,float percent_rejected_ver,float percent_rejected_unver,float avg_accel_diff_ver,float avg_accel_diff_unver)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
	char buf[MAVLINK_MSG_ID_SHIM_STATS_LEN];
	_mav_put_uint64_t(buf, 0, time);
	_mav_put_float(buf, 8, percent_rejected_ver);
	_mav_put_float(buf, 12, percent_rejected_unver);
	_mav_put_float(buf, 16, avg_accel_diff_ver);
	_mav_put_float(buf, 20, avg_accel_diff_unver);

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), buf, MAVLINK_MSG_ID_SHIM_STATS_LEN);
#else
	mavlink_shim_stats_t packet;
	packet.time = time;
	packet.percent_rejected_ver = percent_rejected_ver;
	packet.percent_rejected_unver = percent_rejected_unver;
	packet.avg_accel_diff_ver = avg_accel_diff_ver;
	packet.avg_accel_diff_unver = avg_accel_diff_unver;

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
	return mavlink_msg_shim_stats_pack(system_id, component_id, msg, shim_stats->time, shim_stats->percent_rejected_ver, shim_stats->percent_rejected_unver, shim_stats->avg_accel_diff_ver, shim_stats->avg_accel_diff_unver);
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
	return mavlink_msg_shim_stats_pack_chan(system_id, component_id, chan, msg, shim_stats->time, shim_stats->percent_rejected_ver, shim_stats->percent_rejected_unver, shim_stats->avg_accel_diff_ver, shim_stats->avg_accel_diff_unver);
}

/**
 * @brief Send a shim_stats message
 * @param chan MAVLink channel to send the message
 *
 * @param time The time of these statistics in milliseconds since the epoch.
 * @param percent_rejected_ver The percentage of iterations in the last window_time milliseconds that the verified shim rejected the proposed acceleration and issued its own.
 * @param percent_rejected_unver The percentage of iterations in the last window_time milliseconds that the unverified smoothing shim rejected the proposed acceleration and issued its own.
 * @param avg_accel_diff_ver The average difference in the last window_time milliseconds between proposed acceleration and the acceleration issued by the verified shim.
 * @param avg_accel_diff_unver The average difference in the last window_time milliseconds between proposed acceleration and the acceleration issued by the unverified shim.
 */
#ifdef MAVLINK_USE_CONVENIENCE_FUNCTIONS

static inline void mavlink_msg_shim_stats_send(mavlink_channel_t chan, uint64_t time, float percent_rejected_ver, float percent_rejected_unver, float avg_accel_diff_ver, float avg_accel_diff_unver)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
	char buf[MAVLINK_MSG_ID_SHIM_STATS_LEN];
	_mav_put_uint64_t(buf, 0, time);
	_mav_put_float(buf, 8, percent_rejected_ver);
	_mav_put_float(buf, 12, percent_rejected_unver);
	_mav_put_float(buf, 16, avg_accel_diff_ver);
	_mav_put_float(buf, 20, avg_accel_diff_unver);

#if MAVLINK_CRC_EXTRA
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_SHIM_STATS, buf, MAVLINK_MSG_ID_SHIM_STATS_LEN, MAVLINK_MSG_ID_SHIM_STATS_CRC);
#else
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_SHIM_STATS, buf, MAVLINK_MSG_ID_SHIM_STATS_LEN);
#endif
#else
	mavlink_shim_stats_t packet;
	packet.time = time;
	packet.percent_rejected_ver = percent_rejected_ver;
	packet.percent_rejected_unver = percent_rejected_unver;
	packet.avg_accel_diff_ver = avg_accel_diff_ver;
	packet.avg_accel_diff_unver = avg_accel_diff_unver;

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
static inline void mavlink_msg_shim_stats_send_buf(mavlink_message_t *msgbuf, mavlink_channel_t chan,  uint64_t time, float percent_rejected_ver, float percent_rejected_unver, float avg_accel_diff_ver, float avg_accel_diff_unver)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
	char *buf = (char *)msgbuf;
	_mav_put_uint64_t(buf, 0, time);
	_mav_put_float(buf, 8, percent_rejected_ver);
	_mav_put_float(buf, 12, percent_rejected_unver);
	_mav_put_float(buf, 16, avg_accel_diff_ver);
	_mav_put_float(buf, 20, avg_accel_diff_unver);

#if MAVLINK_CRC_EXTRA
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_SHIM_STATS, buf, MAVLINK_MSG_ID_SHIM_STATS_LEN, MAVLINK_MSG_ID_SHIM_STATS_CRC);
#else
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_SHIM_STATS, buf, MAVLINK_MSG_ID_SHIM_STATS_LEN);
#endif
#else
	mavlink_shim_stats_t *packet = (mavlink_shim_stats_t *)msgbuf;
	packet->time = time;
	packet->percent_rejected_ver = percent_rejected_ver;
	packet->percent_rejected_unver = percent_rejected_unver;
	packet->avg_accel_diff_ver = avg_accel_diff_ver;
	packet->avg_accel_diff_unver = avg_accel_diff_unver;

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
 * @brief Get field time from shim_stats message
 *
 * @return The time of these statistics in milliseconds since the epoch.
 */
static inline uint64_t mavlink_msg_shim_stats_get_time(const mavlink_message_t* msg)
{
	return _MAV_RETURN_uint64_t(msg,  0);
}

/**
 * @brief Get field percent_rejected_ver from shim_stats message
 *
 * @return The percentage of iterations in the last window_time milliseconds that the verified shim rejected the proposed acceleration and issued its own.
 */
static inline float mavlink_msg_shim_stats_get_percent_rejected_ver(const mavlink_message_t* msg)
{
	return _MAV_RETURN_float(msg,  8);
}

/**
 * @brief Get field percent_rejected_unver from shim_stats message
 *
 * @return The percentage of iterations in the last window_time milliseconds that the unverified smoothing shim rejected the proposed acceleration and issued its own.
 */
static inline float mavlink_msg_shim_stats_get_percent_rejected_unver(const mavlink_message_t* msg)
{
	return _MAV_RETURN_float(msg,  12);
}

/**
 * @brief Get field avg_accel_diff_ver from shim_stats message
 *
 * @return The average difference in the last window_time milliseconds between proposed acceleration and the acceleration issued by the verified shim.
 */
static inline float mavlink_msg_shim_stats_get_avg_accel_diff_ver(const mavlink_message_t* msg)
{
	return _MAV_RETURN_float(msg,  16);
}

/**
 * @brief Get field avg_accel_diff_unver from shim_stats message
 *
 * @return The average difference in the last window_time milliseconds between proposed acceleration and the acceleration issued by the unverified shim.
 */
static inline float mavlink_msg_shim_stats_get_avg_accel_diff_unver(const mavlink_message_t* msg)
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
	shim_stats->time = mavlink_msg_shim_stats_get_time(msg);
	shim_stats->percent_rejected_ver = mavlink_msg_shim_stats_get_percent_rejected_ver(msg);
	shim_stats->percent_rejected_unver = mavlink_msg_shim_stats_get_percent_rejected_unver(msg);
	shim_stats->avg_accel_diff_ver = mavlink_msg_shim_stats_get_avg_accel_diff_ver(msg);
	shim_stats->avg_accel_diff_unver = mavlink_msg_shim_stats_get_avg_accel_diff_unver(msg);
#else
	memcpy(shim_stats, _MAV_PAYLOAD(msg), MAVLINK_MSG_ID_SHIM_STATS_LEN);
#endif
}
