// MESSAGE SHIM_PARAMS PACKING

#define MAVLINK_MSG_ID_SHIM_PARAMS 231

typedef struct __mavlink_shim_params_t
{
 float lookahead; ///< lookahead factor for the smoothing shim
 float h_ub; ///< the height upper bound of the shim in cm
 float h_lb; ///< the height lower bound of the shim in cm
 float hprime_ub; ///< the vertical velocity upper bound of the shim in cm/s
 float hprime_lb; ///< the vertical velocity lower bound of the shim in cm/s
 float x_ub; ///< the upper bound for the "x" dimension of the shim in cm
 float x_lb; ///< the lower bound for the "x" dimension of the shim in cm
 float xprime_ub; ///< the "x" dimension velocity upper bound of the shim in cm/s
 float xprime_lb; ///< the "x" dimension velocity lower bound of the shim in cm/s
 float roll_lb; ///< the lower bound on roll angle allowed by the shim in (??DEGREES??)
 float abraking; ///< The braking acceleration in cm/s/s issued when the height-dimension shim is engaged. Should be at least the acceleration due to gravity.
 float mid_throttle; ///< The throttle required for hover.
 uint8_t smooth; ///< 1 to enable smoothing, 0 to disable
} mavlink_shim_params_t;

#define MAVLINK_MSG_ID_SHIM_PARAMS_LEN 49
#define MAVLINK_MSG_ID_231_LEN 49

#define MAVLINK_MSG_ID_SHIM_PARAMS_CRC 189
#define MAVLINK_MSG_ID_231_CRC 189



#define MAVLINK_MESSAGE_INFO_SHIM_PARAMS { \
	"SHIM_PARAMS", \
	13, \
	{  { "lookahead", NULL, MAVLINK_TYPE_FLOAT, 0, 0, offsetof(mavlink_shim_params_t, lookahead) }, \
         { "h_ub", NULL, MAVLINK_TYPE_FLOAT, 0, 4, offsetof(mavlink_shim_params_t, h_ub) }, \
         { "h_lb", NULL, MAVLINK_TYPE_FLOAT, 0, 8, offsetof(mavlink_shim_params_t, h_lb) }, \
         { "hprime_ub", NULL, MAVLINK_TYPE_FLOAT, 0, 12, offsetof(mavlink_shim_params_t, hprime_ub) }, \
         { "hprime_lb", NULL, MAVLINK_TYPE_FLOAT, 0, 16, offsetof(mavlink_shim_params_t, hprime_lb) }, \
         { "x_ub", NULL, MAVLINK_TYPE_FLOAT, 0, 20, offsetof(mavlink_shim_params_t, x_ub) }, \
         { "x_lb", NULL, MAVLINK_TYPE_FLOAT, 0, 24, offsetof(mavlink_shim_params_t, x_lb) }, \
         { "xprime_ub", NULL, MAVLINK_TYPE_FLOAT, 0, 28, offsetof(mavlink_shim_params_t, xprime_ub) }, \
         { "xprime_lb", NULL, MAVLINK_TYPE_FLOAT, 0, 32, offsetof(mavlink_shim_params_t, xprime_lb) }, \
         { "roll_lb", NULL, MAVLINK_TYPE_FLOAT, 0, 36, offsetof(mavlink_shim_params_t, roll_lb) }, \
         { "abraking", NULL, MAVLINK_TYPE_FLOAT, 0, 40, offsetof(mavlink_shim_params_t, abraking) }, \
         { "mid_throttle", NULL, MAVLINK_TYPE_FLOAT, 0, 44, offsetof(mavlink_shim_params_t, mid_throttle) }, \
         { "smooth", NULL, MAVLINK_TYPE_UINT8_T, 0, 48, offsetof(mavlink_shim_params_t, smooth) }, \
         } \
}


/**
 * @brief Pack a shim_params message
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param msg The MAVLink message to compress the data into
 *
 * @param smooth 1 to enable smoothing, 0 to disable
 * @param lookahead lookahead factor for the smoothing shim
 * @param h_ub the height upper bound of the shim in cm
 * @param h_lb the height lower bound of the shim in cm
 * @param hprime_ub the vertical velocity upper bound of the shim in cm/s
 * @param hprime_lb the vertical velocity lower bound of the shim in cm/s
 * @param x_ub the upper bound for the "x" dimension of the shim in cm
 * @param x_lb the lower bound for the "x" dimension of the shim in cm
 * @param xprime_ub the "x" dimension velocity upper bound of the shim in cm/s
 * @param xprime_lb the "x" dimension velocity lower bound of the shim in cm/s
 * @param roll_lb the lower bound on roll angle allowed by the shim in (??DEGREES??)
 * @param abraking The braking acceleration in cm/s/s issued when the height-dimension shim is engaged. Should be at least the acceleration due to gravity.
 * @param mid_throttle The throttle required for hover.
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t mavlink_msg_shim_params_pack(uint8_t system_id, uint8_t component_id, mavlink_message_t* msg,
						       uint8_t smooth, float lookahead, float h_ub, float h_lb, float hprime_ub, float hprime_lb, float x_ub, float x_lb, float xprime_ub, float xprime_lb, float roll_lb, float abraking, float mid_throttle)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
	char buf[MAVLINK_MSG_ID_SHIM_PARAMS_LEN];
	_mav_put_float(buf, 0, lookahead);
	_mav_put_float(buf, 4, h_ub);
	_mav_put_float(buf, 8, h_lb);
	_mav_put_float(buf, 12, hprime_ub);
	_mav_put_float(buf, 16, hprime_lb);
	_mav_put_float(buf, 20, x_ub);
	_mav_put_float(buf, 24, x_lb);
	_mav_put_float(buf, 28, xprime_ub);
	_mav_put_float(buf, 32, xprime_lb);
	_mav_put_float(buf, 36, roll_lb);
	_mav_put_float(buf, 40, abraking);
	_mav_put_float(buf, 44, mid_throttle);
	_mav_put_uint8_t(buf, 48, smooth);

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), buf, MAVLINK_MSG_ID_SHIM_PARAMS_LEN);
#else
	mavlink_shim_params_t packet;
	packet.lookahead = lookahead;
	packet.h_ub = h_ub;
	packet.h_lb = h_lb;
	packet.hprime_ub = hprime_ub;
	packet.hprime_lb = hprime_lb;
	packet.x_ub = x_ub;
	packet.x_lb = x_lb;
	packet.xprime_ub = xprime_ub;
	packet.xprime_lb = xprime_lb;
	packet.roll_lb = roll_lb;
	packet.abraking = abraking;
	packet.mid_throttle = mid_throttle;
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
 * @param smooth 1 to enable smoothing, 0 to disable
 * @param lookahead lookahead factor for the smoothing shim
 * @param h_ub the height upper bound of the shim in cm
 * @param h_lb the height lower bound of the shim in cm
 * @param hprime_ub the vertical velocity upper bound of the shim in cm/s
 * @param hprime_lb the vertical velocity lower bound of the shim in cm/s
 * @param x_ub the upper bound for the "x" dimension of the shim in cm
 * @param x_lb the lower bound for the "x" dimension of the shim in cm
 * @param xprime_ub the "x" dimension velocity upper bound of the shim in cm/s
 * @param xprime_lb the "x" dimension velocity lower bound of the shim in cm/s
 * @param roll_lb the lower bound on roll angle allowed by the shim in (??DEGREES??)
 * @param abraking The braking acceleration in cm/s/s issued when the height-dimension shim is engaged. Should be at least the acceleration due to gravity.
 * @param mid_throttle The throttle required for hover.
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t mavlink_msg_shim_params_pack_chan(uint8_t system_id, uint8_t component_id, uint8_t chan,
							   mavlink_message_t* msg,
						           uint8_t smooth,float lookahead,float h_ub,float h_lb,float hprime_ub,float hprime_lb,float x_ub,float x_lb,float xprime_ub,float xprime_lb,float roll_lb,float abraking,float mid_throttle)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
	char buf[MAVLINK_MSG_ID_SHIM_PARAMS_LEN];
	_mav_put_float(buf, 0, lookahead);
	_mav_put_float(buf, 4, h_ub);
	_mav_put_float(buf, 8, h_lb);
	_mav_put_float(buf, 12, hprime_ub);
	_mav_put_float(buf, 16, hprime_lb);
	_mav_put_float(buf, 20, x_ub);
	_mav_put_float(buf, 24, x_lb);
	_mav_put_float(buf, 28, xprime_ub);
	_mav_put_float(buf, 32, xprime_lb);
	_mav_put_float(buf, 36, roll_lb);
	_mav_put_float(buf, 40, abraking);
	_mav_put_float(buf, 44, mid_throttle);
	_mav_put_uint8_t(buf, 48, smooth);

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), buf, MAVLINK_MSG_ID_SHIM_PARAMS_LEN);
#else
	mavlink_shim_params_t packet;
	packet.lookahead = lookahead;
	packet.h_ub = h_ub;
	packet.h_lb = h_lb;
	packet.hprime_ub = hprime_ub;
	packet.hprime_lb = hprime_lb;
	packet.x_ub = x_ub;
	packet.x_lb = x_lb;
	packet.xprime_ub = xprime_ub;
	packet.xprime_lb = xprime_lb;
	packet.roll_lb = roll_lb;
	packet.abraking = abraking;
	packet.mid_throttle = mid_throttle;
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
	return mavlink_msg_shim_params_pack(system_id, component_id, msg, shim_params->smooth, shim_params->lookahead, shim_params->h_ub, shim_params->h_lb, shim_params->hprime_ub, shim_params->hprime_lb, shim_params->x_ub, shim_params->x_lb, shim_params->xprime_ub, shim_params->xprime_lb, shim_params->roll_lb, shim_params->abraking, shim_params->mid_throttle);
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
	return mavlink_msg_shim_params_pack_chan(system_id, component_id, chan, msg, shim_params->smooth, shim_params->lookahead, shim_params->h_ub, shim_params->h_lb, shim_params->hprime_ub, shim_params->hprime_lb, shim_params->x_ub, shim_params->x_lb, shim_params->xprime_ub, shim_params->xprime_lb, shim_params->roll_lb, shim_params->abraking, shim_params->mid_throttle);
}

/**
 * @brief Send a shim_params message
 * @param chan MAVLink channel to send the message
 *
 * @param smooth 1 to enable smoothing, 0 to disable
 * @param lookahead lookahead factor for the smoothing shim
 * @param h_ub the height upper bound of the shim in cm
 * @param h_lb the height lower bound of the shim in cm
 * @param hprime_ub the vertical velocity upper bound of the shim in cm/s
 * @param hprime_lb the vertical velocity lower bound of the shim in cm/s
 * @param x_ub the upper bound for the "x" dimension of the shim in cm
 * @param x_lb the lower bound for the "x" dimension of the shim in cm
 * @param xprime_ub the "x" dimension velocity upper bound of the shim in cm/s
 * @param xprime_lb the "x" dimension velocity lower bound of the shim in cm/s
 * @param roll_lb the lower bound on roll angle allowed by the shim in (??DEGREES??)
 * @param abraking The braking acceleration in cm/s/s issued when the height-dimension shim is engaged. Should be at least the acceleration due to gravity.
 * @param mid_throttle The throttle required for hover.
 */
#ifdef MAVLINK_USE_CONVENIENCE_FUNCTIONS

static inline void mavlink_msg_shim_params_send(mavlink_channel_t chan, uint8_t smooth, float lookahead, float h_ub, float h_lb, float hprime_ub, float hprime_lb, float x_ub, float x_lb, float xprime_ub, float xprime_lb, float roll_lb, float abraking, float mid_throttle)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
	char buf[MAVLINK_MSG_ID_SHIM_PARAMS_LEN];
	_mav_put_float(buf, 0, lookahead);
	_mav_put_float(buf, 4, h_ub);
	_mav_put_float(buf, 8, h_lb);
	_mav_put_float(buf, 12, hprime_ub);
	_mav_put_float(buf, 16, hprime_lb);
	_mav_put_float(buf, 20, x_ub);
	_mav_put_float(buf, 24, x_lb);
	_mav_put_float(buf, 28, xprime_ub);
	_mav_put_float(buf, 32, xprime_lb);
	_mav_put_float(buf, 36, roll_lb);
	_mav_put_float(buf, 40, abraking);
	_mav_put_float(buf, 44, mid_throttle);
	_mav_put_uint8_t(buf, 48, smooth);

#if MAVLINK_CRC_EXTRA
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_SHIM_PARAMS, buf, MAVLINK_MSG_ID_SHIM_PARAMS_LEN, MAVLINK_MSG_ID_SHIM_PARAMS_CRC);
#else
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_SHIM_PARAMS, buf, MAVLINK_MSG_ID_SHIM_PARAMS_LEN);
#endif
#else
	mavlink_shim_params_t packet;
	packet.lookahead = lookahead;
	packet.h_ub = h_ub;
	packet.h_lb = h_lb;
	packet.hprime_ub = hprime_ub;
	packet.hprime_lb = hprime_lb;
	packet.x_ub = x_ub;
	packet.x_lb = x_lb;
	packet.xprime_ub = xprime_ub;
	packet.xprime_lb = xprime_lb;
	packet.roll_lb = roll_lb;
	packet.abraking = abraking;
	packet.mid_throttle = mid_throttle;
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
static inline void mavlink_msg_shim_params_send_buf(mavlink_message_t *msgbuf, mavlink_channel_t chan,  uint8_t smooth, float lookahead, float h_ub, float h_lb, float hprime_ub, float hprime_lb, float x_ub, float x_lb, float xprime_ub, float xprime_lb, float roll_lb, float abraking, float mid_throttle)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
	char *buf = (char *)msgbuf;
	_mav_put_float(buf, 0, lookahead);
	_mav_put_float(buf, 4, h_ub);
	_mav_put_float(buf, 8, h_lb);
	_mav_put_float(buf, 12, hprime_ub);
	_mav_put_float(buf, 16, hprime_lb);
	_mav_put_float(buf, 20, x_ub);
	_mav_put_float(buf, 24, x_lb);
	_mav_put_float(buf, 28, xprime_ub);
	_mav_put_float(buf, 32, xprime_lb);
	_mav_put_float(buf, 36, roll_lb);
	_mav_put_float(buf, 40, abraking);
	_mav_put_float(buf, 44, mid_throttle);
	_mav_put_uint8_t(buf, 48, smooth);

#if MAVLINK_CRC_EXTRA
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_SHIM_PARAMS, buf, MAVLINK_MSG_ID_SHIM_PARAMS_LEN, MAVLINK_MSG_ID_SHIM_PARAMS_CRC);
#else
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_SHIM_PARAMS, buf, MAVLINK_MSG_ID_SHIM_PARAMS_LEN);
#endif
#else
	mavlink_shim_params_t *packet = (mavlink_shim_params_t *)msgbuf;
	packet->lookahead = lookahead;
	packet->h_ub = h_ub;
	packet->h_lb = h_lb;
	packet->hprime_ub = hprime_ub;
	packet->hprime_lb = hprime_lb;
	packet->x_ub = x_ub;
	packet->x_lb = x_lb;
	packet->xprime_ub = xprime_ub;
	packet->xprime_lb = xprime_lb;
	packet->roll_lb = roll_lb;
	packet->abraking = abraking;
	packet->mid_throttle = mid_throttle;
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
 * @brief Get field smooth from shim_params message
 *
 * @return 1 to enable smoothing, 0 to disable
 */
static inline uint8_t mavlink_msg_shim_params_get_smooth(const mavlink_message_t* msg)
{
	return _MAV_RETURN_uint8_t(msg,  48);
}

/**
 * @brief Get field lookahead from shim_params message
 *
 * @return lookahead factor for the smoothing shim
 */
static inline float mavlink_msg_shim_params_get_lookahead(const mavlink_message_t* msg)
{
	return _MAV_RETURN_float(msg,  0);
}

/**
 * @brief Get field h_ub from shim_params message
 *
 * @return the height upper bound of the shim in cm
 */
static inline float mavlink_msg_shim_params_get_h_ub(const mavlink_message_t* msg)
{
	return _MAV_RETURN_float(msg,  4);
}

/**
 * @brief Get field h_lb from shim_params message
 *
 * @return the height lower bound of the shim in cm
 */
static inline float mavlink_msg_shim_params_get_h_lb(const mavlink_message_t* msg)
{
	return _MAV_RETURN_float(msg,  8);
}

/**
 * @brief Get field hprime_ub from shim_params message
 *
 * @return the vertical velocity upper bound of the shim in cm/s
 */
static inline float mavlink_msg_shim_params_get_hprime_ub(const mavlink_message_t* msg)
{
	return _MAV_RETURN_float(msg,  12);
}

/**
 * @brief Get field hprime_lb from shim_params message
 *
 * @return the vertical velocity lower bound of the shim in cm/s
 */
static inline float mavlink_msg_shim_params_get_hprime_lb(const mavlink_message_t* msg)
{
	return _MAV_RETURN_float(msg,  16);
}

/**
 * @brief Get field x_ub from shim_params message
 *
 * @return the upper bound for the "x" dimension of the shim in cm
 */
static inline float mavlink_msg_shim_params_get_x_ub(const mavlink_message_t* msg)
{
	return _MAV_RETURN_float(msg,  20);
}

/**
 * @brief Get field x_lb from shim_params message
 *
 * @return the lower bound for the "x" dimension of the shim in cm
 */
static inline float mavlink_msg_shim_params_get_x_lb(const mavlink_message_t* msg)
{
	return _MAV_RETURN_float(msg,  24);
}

/**
 * @brief Get field xprime_ub from shim_params message
 *
 * @return the "x" dimension velocity upper bound of the shim in cm/s
 */
static inline float mavlink_msg_shim_params_get_xprime_ub(const mavlink_message_t* msg)
{
	return _MAV_RETURN_float(msg,  28);
}

/**
 * @brief Get field xprime_lb from shim_params message
 *
 * @return the "x" dimension velocity lower bound of the shim in cm/s
 */
static inline float mavlink_msg_shim_params_get_xprime_lb(const mavlink_message_t* msg)
{
	return _MAV_RETURN_float(msg,  32);
}

/**
 * @brief Get field roll_lb from shim_params message
 *
 * @return the lower bound on roll angle allowed by the shim in (??DEGREES??)
 */
static inline float mavlink_msg_shim_params_get_roll_lb(const mavlink_message_t* msg)
{
	return _MAV_RETURN_float(msg,  36);
}

/**
 * @brief Get field abraking from shim_params message
 *
 * @return The braking acceleration in cm/s/s issued when the height-dimension shim is engaged. Should be at least the acceleration due to gravity.
 */
static inline float mavlink_msg_shim_params_get_abraking(const mavlink_message_t* msg)
{
	return _MAV_RETURN_float(msg,  40);
}

/**
 * @brief Get field mid_throttle from shim_params message
 *
 * @return The throttle required for hover.
 */
static inline float mavlink_msg_shim_params_get_mid_throttle(const mavlink_message_t* msg)
{
	return _MAV_RETURN_float(msg,  44);
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
	shim_params->lookahead = mavlink_msg_shim_params_get_lookahead(msg);
	shim_params->h_ub = mavlink_msg_shim_params_get_h_ub(msg);
	shim_params->h_lb = mavlink_msg_shim_params_get_h_lb(msg);
	shim_params->hprime_ub = mavlink_msg_shim_params_get_hprime_ub(msg);
	shim_params->hprime_lb = mavlink_msg_shim_params_get_hprime_lb(msg);
	shim_params->x_ub = mavlink_msg_shim_params_get_x_ub(msg);
	shim_params->x_lb = mavlink_msg_shim_params_get_x_lb(msg);
	shim_params->xprime_ub = mavlink_msg_shim_params_get_xprime_ub(msg);
	shim_params->xprime_lb = mavlink_msg_shim_params_get_xprime_lb(msg);
	shim_params->roll_lb = mavlink_msg_shim_params_get_roll_lb(msg);
	shim_params->abraking = mavlink_msg_shim_params_get_abraking(msg);
	shim_params->mid_throttle = mavlink_msg_shim_params_get_mid_throttle(msg);
	shim_params->smooth = mavlink_msg_shim_params_get_smooth(msg);
#else
	memcpy(shim_params, _MAV_PAYLOAD(msg), MAVLINK_MSG_ID_SHIM_PARAMS_LEN);
#endif
}
