// MESSAGE SHIM_PARAMS PACKING

#define MAVLINK_MSG_ID_SHIM_PARAMS 231

typedef struct __mavlink_shim_params_t
{
 float ubverified; ///< the height upper bound in cm of the verified shim
 float ubunverified; ///< the height upper bound in cm of the unverified smoothing shim
 float amin; ///< The acceleration in cm/s/s issued when the verified shim is engaged. Should be at least the acceleration due to gravity.
 float pwm_accel_scale; ///< The ratio between the signal sent to a motor and the acceleration in cm/s/s produced by that motor
 float vel_ubverified; ///< the z velocity upper bound in cm/s of the verified shim
 float vel_ubunverified; ///< the z velocity upper bound in cm/s of the unverified smoothing shim
 uint16_t hover_throttle; ///< The throttle provided to the motor mixing code required for hover
 uint16_t smooth_lookahead; ///< The number of iterations in the future that the smoothing shim looks.
 uint8_t before; ///< 1 to run the shim before the motor mixing code, 0 for after
 uint8_t smooth; ///< 1 to run the unverified smoothing shim, 0 to not run the unverified smoothing shim
 uint8_t height_shim; ///< 1 to run the height shim, 0 to not run it
 uint8_t vel_shim; ///< 1 to run the velocity shim, 0 to not run it
} mavlink_shim_params_t;

#define MAVLINK_MSG_ID_SHIM_PARAMS_LEN 32
#define MAVLINK_MSG_ID_231_LEN 32

#define MAVLINK_MSG_ID_SHIM_PARAMS_CRC 86
#define MAVLINK_MSG_ID_231_CRC 86



#define MAVLINK_MESSAGE_INFO_SHIM_PARAMS { \
	"SHIM_PARAMS", \
	12, \
	{  { "ubverified", NULL, MAVLINK_TYPE_FLOAT, 0, 0, offsetof(mavlink_shim_params_t, ubverified) }, \
         { "ubunverified", NULL, MAVLINK_TYPE_FLOAT, 0, 4, offsetof(mavlink_shim_params_t, ubunverified) }, \
         { "amin", NULL, MAVLINK_TYPE_FLOAT, 0, 8, offsetof(mavlink_shim_params_t, amin) }, \
         { "pwm_accel_scale", NULL, MAVLINK_TYPE_FLOAT, 0, 12, offsetof(mavlink_shim_params_t, pwm_accel_scale) }, \
         { "vel_ubverified", NULL, MAVLINK_TYPE_FLOAT, 0, 16, offsetof(mavlink_shim_params_t, vel_ubverified) }, \
         { "vel_ubunverified", NULL, MAVLINK_TYPE_FLOAT, 0, 20, offsetof(mavlink_shim_params_t, vel_ubunverified) }, \
         { "hover_throttle", NULL, MAVLINK_TYPE_UINT16_T, 0, 24, offsetof(mavlink_shim_params_t, hover_throttle) }, \
         { "smooth_lookahead", NULL, MAVLINK_TYPE_UINT16_T, 0, 26, offsetof(mavlink_shim_params_t, smooth_lookahead) }, \
         { "before", NULL, MAVLINK_TYPE_UINT8_T, 0, 28, offsetof(mavlink_shim_params_t, before) }, \
         { "smooth", NULL, MAVLINK_TYPE_UINT8_T, 0, 29, offsetof(mavlink_shim_params_t, smooth) }, \
         { "height_shim", NULL, MAVLINK_TYPE_UINT8_T, 0, 30, offsetof(mavlink_shim_params_t, height_shim) }, \
         { "vel_shim", NULL, MAVLINK_TYPE_UINT8_T, 0, 31, offsetof(mavlink_shim_params_t, vel_shim) }, \
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
 * @param hover_throttle The throttle provided to the motor mixing code required for hover
 * @param smooth_lookahead The number of iterations in the future that the smoothing shim looks.
 * @param height_shim 1 to run the height shim, 0 to not run it
 * @param vel_shim 1 to run the velocity shim, 0 to not run it
 * @param vel_ubverified the z velocity upper bound in cm/s of the verified shim
 * @param vel_ubunverified the z velocity upper bound in cm/s of the unverified smoothing shim
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t mavlink_msg_shim_params_pack(uint8_t system_id, uint8_t component_id, mavlink_message_t* msg,
						       uint8_t before, uint8_t smooth, float ubverified, float ubunverified, float amin, float pwm_accel_scale, uint16_t hover_throttle, uint16_t smooth_lookahead, uint8_t height_shim, uint8_t vel_shim, float vel_ubverified, float vel_ubunverified)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
	char buf[MAVLINK_MSG_ID_SHIM_PARAMS_LEN];
	_mav_put_float(buf, 0, ubverified);
	_mav_put_float(buf, 4, ubunverified);
	_mav_put_float(buf, 8, amin);
	_mav_put_float(buf, 12, pwm_accel_scale);
	_mav_put_float(buf, 16, vel_ubverified);
	_mav_put_float(buf, 20, vel_ubunverified);
	_mav_put_uint16_t(buf, 24, hover_throttle);
	_mav_put_uint16_t(buf, 26, smooth_lookahead);
	_mav_put_uint8_t(buf, 28, before);
	_mav_put_uint8_t(buf, 29, smooth);
	_mav_put_uint8_t(buf, 30, height_shim);
	_mav_put_uint8_t(buf, 31, vel_shim);

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), buf, MAVLINK_MSG_ID_SHIM_PARAMS_LEN);
#else
	mavlink_shim_params_t packet;
	packet.ubverified = ubverified;
	packet.ubunverified = ubunverified;
	packet.amin = amin;
	packet.pwm_accel_scale = pwm_accel_scale;
	packet.vel_ubverified = vel_ubverified;
	packet.vel_ubunverified = vel_ubunverified;
	packet.hover_throttle = hover_throttle;
	packet.smooth_lookahead = smooth_lookahead;
	packet.before = before;
	packet.smooth = smooth;
	packet.height_shim = height_shim;
	packet.vel_shim = vel_shim;

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
 * @param hover_throttle The throttle provided to the motor mixing code required for hover
 * @param smooth_lookahead The number of iterations in the future that the smoothing shim looks.
 * @param height_shim 1 to run the height shim, 0 to not run it
 * @param vel_shim 1 to run the velocity shim, 0 to not run it
 * @param vel_ubverified the z velocity upper bound in cm/s of the verified shim
 * @param vel_ubunverified the z velocity upper bound in cm/s of the unverified smoothing shim
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t mavlink_msg_shim_params_pack_chan(uint8_t system_id, uint8_t component_id, uint8_t chan,
							   mavlink_message_t* msg,
						           uint8_t before,uint8_t smooth,float ubverified,float ubunverified,float amin,float pwm_accel_scale,uint16_t hover_throttle,uint16_t smooth_lookahead,uint8_t height_shim,uint8_t vel_shim,float vel_ubverified,float vel_ubunverified)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
	char buf[MAVLINK_MSG_ID_SHIM_PARAMS_LEN];
	_mav_put_float(buf, 0, ubverified);
	_mav_put_float(buf, 4, ubunverified);
	_mav_put_float(buf, 8, amin);
	_mav_put_float(buf, 12, pwm_accel_scale);
	_mav_put_float(buf, 16, vel_ubverified);
	_mav_put_float(buf, 20, vel_ubunverified);
	_mav_put_uint16_t(buf, 24, hover_throttle);
	_mav_put_uint16_t(buf, 26, smooth_lookahead);
	_mav_put_uint8_t(buf, 28, before);
	_mav_put_uint8_t(buf, 29, smooth);
	_mav_put_uint8_t(buf, 30, height_shim);
	_mav_put_uint8_t(buf, 31, vel_shim);

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), buf, MAVLINK_MSG_ID_SHIM_PARAMS_LEN);
#else
	mavlink_shim_params_t packet;
	packet.ubverified = ubverified;
	packet.ubunverified = ubunverified;
	packet.amin = amin;
	packet.pwm_accel_scale = pwm_accel_scale;
	packet.vel_ubverified = vel_ubverified;
	packet.vel_ubunverified = vel_ubunverified;
	packet.hover_throttle = hover_throttle;
	packet.smooth_lookahead = smooth_lookahead;
	packet.before = before;
	packet.smooth = smooth;
	packet.height_shim = height_shim;
	packet.vel_shim = vel_shim;

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
	return mavlink_msg_shim_params_pack(system_id, component_id, msg, shim_params->before, shim_params->smooth, shim_params->ubverified, shim_params->ubunverified, shim_params->amin, shim_params->pwm_accel_scale, shim_params->hover_throttle, shim_params->smooth_lookahead, shim_params->height_shim, shim_params->vel_shim, shim_params->vel_ubverified, shim_params->vel_ubunverified);
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
	return mavlink_msg_shim_params_pack_chan(system_id, component_id, chan, msg, shim_params->before, shim_params->smooth, shim_params->ubverified, shim_params->ubunverified, shim_params->amin, shim_params->pwm_accel_scale, shim_params->hover_throttle, shim_params->smooth_lookahead, shim_params->height_shim, shim_params->vel_shim, shim_params->vel_ubverified, shim_params->vel_ubunverified);
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
 * @param hover_throttle The throttle provided to the motor mixing code required for hover
 * @param smooth_lookahead The number of iterations in the future that the smoothing shim looks.
 * @param height_shim 1 to run the height shim, 0 to not run it
 * @param vel_shim 1 to run the velocity shim, 0 to not run it
 * @param vel_ubverified the z velocity upper bound in cm/s of the verified shim
 * @param vel_ubunverified the z velocity upper bound in cm/s of the unverified smoothing shim
 */
#ifdef MAVLINK_USE_CONVENIENCE_FUNCTIONS

static inline void mavlink_msg_shim_params_send(mavlink_channel_t chan, uint8_t before, uint8_t smooth, float ubverified, float ubunverified, float amin, float pwm_accel_scale, uint16_t hover_throttle, uint16_t smooth_lookahead, uint8_t height_shim, uint8_t vel_shim, float vel_ubverified, float vel_ubunverified)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
	char buf[MAVLINK_MSG_ID_SHIM_PARAMS_LEN];
	_mav_put_float(buf, 0, ubverified);
	_mav_put_float(buf, 4, ubunverified);
	_mav_put_float(buf, 8, amin);
	_mav_put_float(buf, 12, pwm_accel_scale);
	_mav_put_float(buf, 16, vel_ubverified);
	_mav_put_float(buf, 20, vel_ubunverified);
	_mav_put_uint16_t(buf, 24, hover_throttle);
	_mav_put_uint16_t(buf, 26, smooth_lookahead);
	_mav_put_uint8_t(buf, 28, before);
	_mav_put_uint8_t(buf, 29, smooth);
	_mav_put_uint8_t(buf, 30, height_shim);
	_mav_put_uint8_t(buf, 31, vel_shim);

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
	packet.vel_ubverified = vel_ubverified;
	packet.vel_ubunverified = vel_ubunverified;
	packet.hover_throttle = hover_throttle;
	packet.smooth_lookahead = smooth_lookahead;
	packet.before = before;
	packet.smooth = smooth;
	packet.height_shim = height_shim;
	packet.vel_shim = vel_shim;

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
static inline void mavlink_msg_shim_params_send_buf(mavlink_message_t *msgbuf, mavlink_channel_t chan,  uint8_t before, uint8_t smooth, float ubverified, float ubunverified, float amin, float pwm_accel_scale, uint16_t hover_throttle, uint16_t smooth_lookahead, uint8_t height_shim, uint8_t vel_shim, float vel_ubverified, float vel_ubunverified)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
	char *buf = (char *)msgbuf;
	_mav_put_float(buf, 0, ubverified);
	_mav_put_float(buf, 4, ubunverified);
	_mav_put_float(buf, 8, amin);
	_mav_put_float(buf, 12, pwm_accel_scale);
	_mav_put_float(buf, 16, vel_ubverified);
	_mav_put_float(buf, 20, vel_ubunverified);
	_mav_put_uint16_t(buf, 24, hover_throttle);
	_mav_put_uint16_t(buf, 26, smooth_lookahead);
	_mav_put_uint8_t(buf, 28, before);
	_mav_put_uint8_t(buf, 29, smooth);
	_mav_put_uint8_t(buf, 30, height_shim);
	_mav_put_uint8_t(buf, 31, vel_shim);

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
	packet->vel_ubverified = vel_ubverified;
	packet->vel_ubunverified = vel_ubunverified;
	packet->hover_throttle = hover_throttle;
	packet->smooth_lookahead = smooth_lookahead;
	packet->before = before;
	packet->smooth = smooth;
	packet->height_shim = height_shim;
	packet->vel_shim = vel_shim;

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
	return _MAV_RETURN_uint8_t(msg,  28);
}

/**
 * @brief Get field smooth from shim_params message
 *
 * @return 1 to run the unverified smoothing shim, 0 to not run the unverified smoothing shim
 */
static inline uint8_t mavlink_msg_shim_params_get_smooth(const mavlink_message_t* msg)
{
	return _MAV_RETURN_uint8_t(msg,  29);
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
 * @brief Get field hover_throttle from shim_params message
 *
 * @return The throttle provided to the motor mixing code required for hover
 */
static inline uint16_t mavlink_msg_shim_params_get_hover_throttle(const mavlink_message_t* msg)
{
	return _MAV_RETURN_uint16_t(msg,  24);
}

/**
 * @brief Get field smooth_lookahead from shim_params message
 *
 * @return The number of iterations in the future that the smoothing shim looks.
 */
static inline uint16_t mavlink_msg_shim_params_get_smooth_lookahead(const mavlink_message_t* msg)
{
	return _MAV_RETURN_uint16_t(msg,  26);
}

/**
 * @brief Get field height_shim from shim_params message
 *
 * @return 1 to run the height shim, 0 to not run it
 */
static inline uint8_t mavlink_msg_shim_params_get_height_shim(const mavlink_message_t* msg)
{
	return _MAV_RETURN_uint8_t(msg,  30);
}

/**
 * @brief Get field vel_shim from shim_params message
 *
 * @return 1 to run the velocity shim, 0 to not run it
 */
static inline uint8_t mavlink_msg_shim_params_get_vel_shim(const mavlink_message_t* msg)
{
	return _MAV_RETURN_uint8_t(msg,  31);
}

/**
 * @brief Get field vel_ubverified from shim_params message
 *
 * @return the z velocity upper bound in cm/s of the verified shim
 */
static inline float mavlink_msg_shim_params_get_vel_ubverified(const mavlink_message_t* msg)
{
	return _MAV_RETURN_float(msg,  16);
}

/**
 * @brief Get field vel_ubunverified from shim_params message
 *
 * @return the z velocity upper bound in cm/s of the unverified smoothing shim
 */
static inline float mavlink_msg_shim_params_get_vel_ubunverified(const mavlink_message_t* msg)
{
	return _MAV_RETURN_float(msg,  20);
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
	shim_params->vel_ubverified = mavlink_msg_shim_params_get_vel_ubverified(msg);
	shim_params->vel_ubunverified = mavlink_msg_shim_params_get_vel_ubunverified(msg);
	shim_params->hover_throttle = mavlink_msg_shim_params_get_hover_throttle(msg);
	shim_params->smooth_lookahead = mavlink_msg_shim_params_get_smooth_lookahead(msg);
	shim_params->before = mavlink_msg_shim_params_get_before(msg);
	shim_params->smooth = mavlink_msg_shim_params_get_smooth(msg);
	shim_params->height_shim = mavlink_msg_shim_params_get_height_shim(msg);
	shim_params->vel_shim = mavlink_msg_shim_params_get_vel_shim(msg);
#else
	memcpy(shim_params, _MAV_PAYLOAD(msg), MAVLINK_MSG_ID_SHIM_PARAMS_LEN);
#endif
}
