// MESSAGE SHIM_PARAMS PACKING

#define MAVLINK_MSG_ID_SHIM_PARAMS 231

typedef struct __mavlink_shim_params_t
{
 float ubverified; ///< the height upper bound in cm of the verified shim
 float ubunverified; ///< the height upper bound in cm of the unverified smoothing shim
 float amin; ///< The acceleration in cm/s/s issued when the verified shim is engaged. Should be at least the acceleration due to gravity.
 float pwm_accel_scale; ///< The ratio between the signal sent to a motor and the acceleration in cm/s/s produced by that motor
 float throttle_accel_scale; ///< The ratio between the throttle issued to the motor mixing code and the acceleration in cm/s/s produced by that motor
 uint32_t window_time; ///< The shim sends various values to the GCS that are computed over the last n time. For example, the percent of time the safety mode of the shim was engaged. This field gives the amount of time in milliseconds over which these values are computed.
 uint8_t before; ///< 1 to run the shim before the motor mixing code, 0 for after
 uint8_t smooth; ///< 1 to run the unverified smoothing shim, 0 to not run the unverified smoothing shim
} mavlink_shim_params_t;

#define MAVLINK_MSG_ID_SHIM_PARAMS_LEN 26
#define MAVLINK_MSG_ID_231_LEN 26

#define MAVLINK_MSG_ID_SHIM_PARAMS_CRC 235
#define MAVLINK_MSG_ID_231_CRC 235



#define MAVLINK_MESSAGE_INFO_SHIM_PARAMS { \
	"SHIM_PARAMS", \
	8, \
	{  { "ubverified", NULL, MAVLINK_TYPE_FLOAT, 0, 0, offsetof(mavlink_shim_params_t, ubverified) }, \
         { "ubunverified", NULL, MAVLINK_TYPE_FLOAT, 0, 4, offsetof(mavlink_shim_params_t, ubunverified) }, \
         { "amin", NULL, MAVLINK_TYPE_FLOAT, 0, 8, offsetof(mavlink_shim_params_t, amin) }, \
         { "pwm_accel_scale", NULL, MAVLINK_TYPE_FLOAT, 0, 12, offsetof(mavlink_shim_params_t, pwm_accel_scale) }, \
         { "throttle_accel_scale", NULL, MAVLINK_TYPE_FLOAT, 0, 16, offsetof(mavlink_shim_params_t, throttle_accel_scale) }, \
         { "window_time", NULL, MAVLINK_TYPE_UINT32_T, 0, 20, offsetof(mavlink_shim_params_t, window_time) }, \
         { "before", NULL, MAVLINK_TYPE_UINT8_T, 0, 24, offsetof(mavlink_shim_params_t, before) }, \
         { "smooth", NULL, MAVLINK_TYPE_UINT8_T, 0, 25, offsetof(mavlink_shim_params_t, smooth) }, \
         } \
}


/**
 * @brief Pack a shim_params message
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param msg The MAVLink message to compress the data into
 *
 * @param before 1 to run the shim before the motor mixing code, 0 for after
 * @param smooth 1 to run the unverified smoothing shim, 0 to not run the unverified smoothing shim
 * @param ubverified the height upper bound in cm of the verified shim
 * @param ubunverified the height upper bound in cm of the unverified smoothing shim
 * @param amin The acceleration in cm/s/s issued when the verified shim is engaged. Should be at least the acceleration due to gravity.
 * @param pwm_accel_scale The ratio between the signal sent to a motor and the acceleration in cm/s/s produced by that motor
 * @param throttle_accel_scale The ratio between the throttle issued to the motor mixing code and the acceleration in cm/s/s produced by that motor
 * @param window_time The shim sends various values to the GCS that are computed over the last n time. For example, the percent of time the safety mode of the shim was engaged. This field gives the amount of time in milliseconds over which these values are computed.
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t mavlink_msg_shim_params_pack(uint8_t system_id, uint8_t component_id, mavlink_message_t* msg,
						       uint8_t before, uint8_t smooth, float ubverified, float ubunverified, float amin, float pwm_accel_scale, float throttle_accel_scale, uint32_t window_time)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
	char buf[MAVLINK_MSG_ID_SHIM_PARAMS_LEN];
	_mav_put_float(buf, 0, ubverified);
	_mav_put_float(buf, 4, ubunverified);
	_mav_put_float(buf, 8, amin);
	_mav_put_float(buf, 12, pwm_accel_scale);
	_mav_put_float(buf, 16, throttle_accel_scale);
	_mav_put_uint32_t(buf, 20, window_time);
	_mav_put_uint8_t(buf, 24, before);
	_mav_put_uint8_t(buf, 25, smooth);

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), buf, MAVLINK_MSG_ID_SHIM_PARAMS_LEN);
#else
	mavlink_shim_params_t packet;
	packet.ubverified = ubverified;
	packet.ubunverified = ubunverified;
	packet.amin = amin;
	packet.pwm_accel_scale = pwm_accel_scale;
	packet.throttle_accel_scale = throttle_accel_scale;
	packet.window_time = window_time;
	packet.before = before;
	packet.smooth = smooth;

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), &packet, MAVLINK_MSG_ID_SHIM_PARAMS_LEN);
#endif

	msg->msgid = MAVLINK_MSG_ID_SHIM_PARAMS;
#if MAVLINK_CRC_EXTRA
    return mavlink_finalize_message(msg, system_id, component_id, MAVLINK_MSG_ID_SHIM_PARAMS_LEN, MAVLINK_MSG_ID_SHIM_PARAMS_CRC);
#else
    return mavlink_finalize_message(msg, system_id, component_id, MAVLINK_MSG_ID_SHIM_PARAMS_LEN);
#endif
}

/**
 * @brief Pack a shim_params message on a channel
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param chan The MAVLink channel this message will be sent over
 * @param msg The MAVLink message to compress the data into
 * @param before 1 to run the shim before the motor mixing code, 0 for after
 * @param smooth 1 to run the unverified smoothing shim, 0 to not run the unverified smoothing shim
 * @param ubverified the height upper bound in cm of the verified shim
 * @param ubunverified the height upper bound in cm of the unverified smoothing shim
 * @param amin The acceleration in cm/s/s issued when the verified shim is engaged. Should be at least the acceleration due to gravity.
 * @param pwm_accel_scale The ratio between the signal sent to a motor and the acceleration in cm/s/s produced by that motor
 * @param throttle_accel_scale The ratio between the throttle issued to the motor mixing code and the acceleration in cm/s/s produced by that motor
 * @param window_time The shim sends various values to the GCS that are computed over the last n time. For example, the percent of time the safety mode of the shim was engaged. This field gives the amount of time in milliseconds over which these values are computed.
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t mavlink_msg_shim_params_pack_chan(uint8_t system_id, uint8_t component_id, uint8_t chan,
							   mavlink_message_t* msg,
						           uint8_t before,uint8_t smooth,float ubverified,float ubunverified,float amin,float pwm_accel_scale,float throttle_accel_scale,uint32_t window_time)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
	char buf[MAVLINK_MSG_ID_SHIM_PARAMS_LEN];
	_mav_put_float(buf, 0, ubverified);
	_mav_put_float(buf, 4, ubunverified);
	_mav_put_float(buf, 8, amin);
	_mav_put_float(buf, 12, pwm_accel_scale);
	_mav_put_float(buf, 16, throttle_accel_scale);
	_mav_put_uint32_t(buf, 20, window_time);
	_mav_put_uint8_t(buf, 24, before);
	_mav_put_uint8_t(buf, 25, smooth);

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), buf, MAVLINK_MSG_ID_SHIM_PARAMS_LEN);
#else
	mavlink_shim_params_t packet;
	packet.ubverified = ubverified;
	packet.ubunverified = ubunverified;
	packet.amin = amin;
	packet.pwm_accel_scale = pwm_accel_scale;
	packet.throttle_accel_scale = throttle_accel_scale;
	packet.window_time = window_time;
	packet.before = before;
	packet.smooth = smooth;

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), &packet, MAVLINK_MSG_ID_SHIM_PARAMS_LEN);
#endif

	msg->msgid = MAVLINK_MSG_ID_SHIM_PARAMS;
#if MAVLINK_CRC_EXTRA
    return mavlink_finalize_message_chan(msg, system_id, component_id, chan, MAVLINK_MSG_ID_SHIM_PARAMS_LEN, MAVLINK_MSG_ID_SHIM_PARAMS_CRC);
#else
    return mavlink_finalize_message_chan(msg, system_id, component_id, chan, MAVLINK_MSG_ID_SHIM_PARAMS_LEN);
#endif
}

/**
 * @brief Encode a shim_params struct
 *
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param msg The MAVLink message to compress the data into
 * @param shim_params C-struct to read the message contents from
 */
static inline uint16_t mavlink_msg_shim_params_encode(uint8_t system_id, uint8_t component_id, mavlink_message_t* msg, const mavlink_shim_params_t* shim_params)
{
	return mavlink_msg_shim_params_pack(system_id, component_id, msg, shim_params->before, shim_params->smooth, shim_params->ubverified, shim_params->ubunverified, shim_params->amin, shim_params->pwm_accel_scale, shim_params->throttle_accel_scale, shim_params->window_time);
}

/**
 * @brief Encode a shim_params struct on a channel
 *
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param chan The MAVLink channel this message will be sent over
 * @param msg The MAVLink message to compress the data into
 * @param shim_params C-struct to read the message contents from
 */
static inline uint16_t mavlink_msg_shim_params_encode_chan(uint8_t system_id, uint8_t component_id, uint8_t chan, mavlink_message_t* msg, const mavlink_shim_params_t* shim_params)
{
	return mavlink_msg_shim_params_pack_chan(system_id, component_id, chan, msg, shim_params->before, shim_params->smooth, shim_params->ubverified, shim_params->ubunverified, shim_params->amin, shim_params->pwm_accel_scale, shim_params->throttle_accel_scale, shim_params->window_time);
}

/**
 * @brief Send a shim_params message
 * @param chan MAVLink channel to send the message
 *
 * @param before 1 to run the shim before the motor mixing code, 0 for after
 * @param smooth 1 to run the unverified smoothing shim, 0 to not run the unverified smoothing shim
 * @param ubverified the height upper bound in cm of the verified shim
 * @param ubunverified the height upper bound in cm of the unverified smoothing shim
 * @param amin The acceleration in cm/s/s issued when the verified shim is engaged. Should be at least the acceleration due to gravity.
 * @param pwm_accel_scale The ratio between the signal sent to a motor and the acceleration in cm/s/s produced by that motor
 * @param throttle_accel_scale The ratio between the throttle issued to the motor mixing code and the acceleration in cm/s/s produced by that motor
 * @param window_time The shim sends various values to the GCS that are computed over the last n time. For example, the percent of time the safety mode of the shim was engaged. This field gives the amount of time in milliseconds over which these values are computed.
 */
#ifdef MAVLINK_USE_CONVENIENCE_FUNCTIONS

static inline void mavlink_msg_shim_params_send(mavlink_channel_t chan, uint8_t before, uint8_t smooth, float ubverified, float ubunverified, float amin, float pwm_accel_scale, float throttle_accel_scale, uint32_t window_time)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
	char buf[MAVLINK_MSG_ID_SHIM_PARAMS_LEN];
	_mav_put_float(buf, 0, ubverified);
	_mav_put_float(buf, 4, ubunverified);
	_mav_put_float(buf, 8, amin);
	_mav_put_float(buf, 12, pwm_accel_scale);
	_mav_put_float(buf, 16, throttle_accel_scale);
	_mav_put_uint32_t(buf, 20, window_time);
	_mav_put_uint8_t(buf, 24, before);
	_mav_put_uint8_t(buf, 25, smooth);

#if MAVLINK_CRC_EXTRA
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_SHIM_PARAMS, buf, MAVLINK_MSG_ID_SHIM_PARAMS_LEN, MAVLINK_MSG_ID_SHIM_PARAMS_CRC);
#else
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_SHIM_PARAMS, buf, MAVLINK_MSG_ID_SHIM_PARAMS_LEN);
#endif
#else
	mavlink_shim_params_t packet;
	packet.ubverified = ubverified;
	packet.ubunverified = ubunverified;
	packet.amin = amin;
	packet.pwm_accel_scale = pwm_accel_scale;
	packet.throttle_accel_scale = throttle_accel_scale;
	packet.window_time = window_time;
	packet.before = before;
	packet.smooth = smooth;

#if MAVLINK_CRC_EXTRA
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_SHIM_PARAMS, (const char *)&packet, MAVLINK_MSG_ID_SHIM_PARAMS_LEN, MAVLINK_MSG_ID_SHIM_PARAMS_CRC);
#else
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_SHIM_PARAMS, (const char *)&packet, MAVLINK_MSG_ID_SHIM_PARAMS_LEN);
#endif
#endif
}

#if MAVLINK_MSG_ID_SHIM_PARAMS_LEN <= MAVLINK_MAX_PAYLOAD_LEN
/*
  This varient of _send() can be used to save stack space by re-using
  memory from the receive buffer.  The caller provides a
  mavlink_message_t which is the size of a full mavlink message. This
  is usually the receive buffer for the channel, and allows a reply to an
  incoming message with minimum stack space usage.
 */
static inline void mavlink_msg_shim_params_send_buf(mavlink_message_t *msgbuf, mavlink_channel_t chan,  uint8_t before, uint8_t smooth, float ubverified, float ubunverified, float amin, float pwm_accel_scale, float throttle_accel_scale, uint32_t window_time)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
	char *buf = (char *)msgbuf;
	_mav_put_float(buf, 0, ubverified);
	_mav_put_float(buf, 4, ubunverified);
	_mav_put_float(buf, 8, amin);
	_mav_put_float(buf, 12, pwm_accel_scale);
	_mav_put_float(buf, 16, throttle_accel_scale);
	_mav_put_uint32_t(buf, 20, window_time);
	_mav_put_uint8_t(buf, 24, before);
	_mav_put_uint8_t(buf, 25, smooth);

#if MAVLINK_CRC_EXTRA
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_SHIM_PARAMS, buf, MAVLINK_MSG_ID_SHIM_PARAMS_LEN, MAVLINK_MSG_ID_SHIM_PARAMS_CRC);
#else
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_SHIM_PARAMS, buf, MAVLINK_MSG_ID_SHIM_PARAMS_LEN);
#endif
#else
	mavlink_shim_params_t *packet = (mavlink_shim_params_t *)msgbuf;
	packet->ubverified = ubverified;
	packet->ubunverified = ubunverified;
	packet->amin = amin;
	packet->pwm_accel_scale = pwm_accel_scale;
	packet->throttle_accel_scale = throttle_accel_scale;
	packet->window_time = window_time;
	packet->before = before;
	packet->smooth = smooth;

#if MAVLINK_CRC_EXTRA
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_SHIM_PARAMS, (const char *)packet, MAVLINK_MSG_ID_SHIM_PARAMS_LEN, MAVLINK_MSG_ID_SHIM_PARAMS_CRC);
#else
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_SHIM_PARAMS, (const char *)packet, MAVLINK_MSG_ID_SHIM_PARAMS_LEN);
#endif
#endif
}
#endif

#endif

// MESSAGE SHIM_PARAMS UNPACKING


/**
 * @brief Get field before from shim_params message
 *
 * @return 1 to run the shim before the motor mixing code, 0 for after
 */
static inline uint8_t mavlink_msg_shim_params_get_before(const mavlink_message_t* msg)
{
	return _MAV_RETURN_uint8_t(msg,  24);
}

/**
 * @brief Get field smooth from shim_params message
 *
 * @return 1 to run the unverified smoothing shim, 0 to not run the unverified smoothing shim
 */
static inline uint8_t mavlink_msg_shim_params_get_smooth(const mavlink_message_t* msg)
{
	return _MAV_RETURN_uint8_t(msg,  25);
}

/**
 * @brief Get field ubverified from shim_params message
 *
 * @return the height upper bound in cm of the verified shim
 */
static inline float mavlink_msg_shim_params_get_ubverified(const mavlink_message_t* msg)
{
	return _MAV_RETURN_float(msg,  0);
}

/**
 * @brief Get field ubunverified from shim_params message
 *
 * @return the height upper bound in cm of the unverified smoothing shim
 */
static inline float mavlink_msg_shim_params_get_ubunverified(const mavlink_message_t* msg)
{
	return _MAV_RETURN_float(msg,  4);
}

/**
 * @brief Get field amin from shim_params message
 *
 * @return The acceleration in cm/s/s issued when the verified shim is engaged. Should be at least the acceleration due to gravity.
 */
static inline float mavlink_msg_shim_params_get_amin(const mavlink_message_t* msg)
{
	return _MAV_RETURN_float(msg,  8);
}

/**
 * @brief Get field pwm_accel_scale from shim_params message
 *
 * @return The ratio between the signal sent to a motor and the acceleration in cm/s/s produced by that motor
 */
static inline float mavlink_msg_shim_params_get_pwm_accel_scale(const mavlink_message_t* msg)
{
	return _MAV_RETURN_float(msg,  12);
}

/**
 * @brief Get field throttle_accel_scale from shim_params message
 *
 * @return The ratio between the throttle issued to the motor mixing code and the acceleration in cm/s/s produced by that motor
 */
static inline float mavlink_msg_shim_params_get_throttle_accel_scale(const mavlink_message_t* msg)
{
	return _MAV_RETURN_float(msg,  16);
}

/**
 * @brief Get field window_time from shim_params message
 *
 * @return The shim sends various values to the GCS that are computed over the last n time. For example, the percent of time the safety mode of the shim was engaged. This field gives the amount of time in milliseconds over which these values are computed.
 */
static inline uint32_t mavlink_msg_shim_params_get_window_time(const mavlink_message_t* msg)
{
	return _MAV_RETURN_uint32_t(msg,  20);
}

/**
 * @brief Decode a shim_params message into a struct
 *
 * @param msg The message to decode
 * @param shim_params C-struct to decode the message contents into
 */
static inline void mavlink_msg_shim_params_decode(const mavlink_message_t* msg, mavlink_shim_params_t* shim_params)
{
#if MAVLINK_NEED_BYTE_SWAP
	shim_params->ubverified = mavlink_msg_shim_params_get_ubverified(msg);
	shim_params->ubunverified = mavlink_msg_shim_params_get_ubunverified(msg);
	shim_params->amin = mavlink_msg_shim_params_get_amin(msg);
	shim_params->pwm_accel_scale = mavlink_msg_shim_params_get_pwm_accel_scale(msg);
	shim_params->throttle_accel_scale = mavlink_msg_shim_params_get_throttle_accel_scale(msg);
	shim_params->window_time = mavlink_msg_shim_params_get_window_time(msg);
	shim_params->before = mavlink_msg_shim_params_get_before(msg);
	shim_params->smooth = mavlink_msg_shim_params_get_smooth(msg);
#else
	memcpy(shim_params, _MAV_PAYLOAD(msg), MAVLINK_MSG_ID_SHIM_PARAMS_LEN);
#endif
}
