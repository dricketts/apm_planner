// MESSAGE SHIM_STATS PACKING

#define MAVLINK_MSG_ID_SHIM_STATS 232

typedef struct __mavlink_shim_stats_t
{
 float x; ///< The value of x, relative to starting position.
 float y; ///< The height, relative to starting position.
 float vx; ///< The velocity in the x direction.
 float vy; ///< The velocity in the vertical direction.
 float A_proposed; ///< The proposed acceleration, normal to the quadcopter.
 float Theta_proposed; ///< The proposed angle
 float a; ///< The issued acceleration, normal to the quadcopter.
 float theta; ///< The issued angle
 uint16_t throttle; ///< The proposed throttle received by the shim
 uint8_t can_run1; ///< 1 if shim 1 can run, 0 otherwise
 uint8_t can_run2; ///< 1 if shim 2 can run, 0 otherwise
 uint8_t can_run3; ///< 1 if shim 3 can run, 0 otherwise
 uint8_t can_run4; ///< 1 if shim 4 can run, 0 otherwise
 uint8_t angle_boost; ///< 1 if angle boost was requested, 0 otherwise
} mavlink_shim_stats_t;

#define MAVLINK_MSG_ID_SHIM_STATS_LEN 39
#define MAVLINK_MSG_ID_232_LEN 39

#define MAVLINK_MSG_ID_SHIM_STATS_CRC 244
#define MAVLINK_MSG_ID_232_CRC 244



#define MAVLINK_MESSAGE_INFO_SHIM_STATS { \
	"SHIM_STATS", \
	14, \
	{  { "x", NULL, MAVLINK_TYPE_FLOAT, 0, 0, offsetof(mavlink_shim_stats_t, x) }, \
         { "y", NULL, MAVLINK_TYPE_FLOAT, 0, 4, offsetof(mavlink_shim_stats_t, y) }, \
         { "vx", NULL, MAVLINK_TYPE_FLOAT, 0, 8, offsetof(mavlink_shim_stats_t, vx) }, \
         { "vy", NULL, MAVLINK_TYPE_FLOAT, 0, 12, offsetof(mavlink_shim_stats_t, vy) }, \
         { "A_proposed", NULL, MAVLINK_TYPE_FLOAT, 0, 16, offsetof(mavlink_shim_stats_t, A_proposed) }, \
         { "Theta_proposed", NULL, MAVLINK_TYPE_FLOAT, 0, 20, offsetof(mavlink_shim_stats_t, Theta_proposed) }, \
         { "a", NULL, MAVLINK_TYPE_FLOAT, 0, 24, offsetof(mavlink_shim_stats_t, a) }, \
         { "theta", NULL, MAVLINK_TYPE_FLOAT, 0, 28, offsetof(mavlink_shim_stats_t, theta) }, \
         { "throttle", NULL, MAVLINK_TYPE_UINT16_T, 0, 32, offsetof(mavlink_shim_stats_t, throttle) }, \
         { "can_run1", NULL, MAVLINK_TYPE_UINT8_T, 0, 34, offsetof(mavlink_shim_stats_t, can_run1) }, \
         { "can_run2", NULL, MAVLINK_TYPE_UINT8_T, 0, 35, offsetof(mavlink_shim_stats_t, can_run2) }, \
         { "can_run3", NULL, MAVLINK_TYPE_UINT8_T, 0, 36, offsetof(mavlink_shim_stats_t, can_run3) }, \
         { "can_run4", NULL, MAVLINK_TYPE_UINT8_T, 0, 37, offsetof(mavlink_shim_stats_t, can_run4) }, \
         { "angle_boost", NULL, MAVLINK_TYPE_UINT8_T, 0, 38, offsetof(mavlink_shim_stats_t, angle_boost) }, \
         } \
}


/**
 * @brief Pack a shim_stats message
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param msg The MAVLink message to compress the data into
 *
 * @param can_run1 1 if shim 1 can run, 0 otherwise
 * @param can_run2 1 if shim 2 can run, 0 otherwise
 * @param can_run3 1 if shim 3 can run, 0 otherwise
 * @param can_run4 1 if shim 4 can run, 0 otherwise
 * @param x The value of x, relative to starting position.
 * @param y The height, relative to starting position.
 * @param vx The velocity in the x direction.
 * @param vy The velocity in the vertical direction.
 * @param throttle The proposed throttle received by the shim
 * @param angle_boost 1 if angle boost was requested, 0 otherwise
 * @param A_proposed The proposed acceleration, normal to the quadcopter.
 * @param Theta_proposed The proposed angle
 * @param a The issued acceleration, normal to the quadcopter.
 * @param theta The issued angle
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t mavlink_msg_shim_stats_pack(uint8_t system_id, uint8_t component_id, mavlink_message_t* msg,
						       uint8_t can_run1, uint8_t can_run2, uint8_t can_run3, uint8_t can_run4, float x, float y, float vx, float vy, uint16_t throttle, uint8_t angle_boost, float A_proposed, float Theta_proposed, float a, float theta)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
	char buf[MAVLINK_MSG_ID_SHIM_STATS_LEN];
	_mav_put_float(buf, 0, x);
	_mav_put_float(buf, 4, y);
	_mav_put_float(buf, 8, vx);
	_mav_put_float(buf, 12, vy);
	_mav_put_float(buf, 16, A_proposed);
	_mav_put_float(buf, 20, Theta_proposed);
	_mav_put_float(buf, 24, a);
	_mav_put_float(buf, 28, theta);
	_mav_put_uint16_t(buf, 32, throttle);
	_mav_put_uint8_t(buf, 34, can_run1);
	_mav_put_uint8_t(buf, 35, can_run2);
	_mav_put_uint8_t(buf, 36, can_run3);
	_mav_put_uint8_t(buf, 37, can_run4);
	_mav_put_uint8_t(buf, 38, angle_boost);

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), buf, MAVLINK_MSG_ID_SHIM_STATS_LEN);
#else
	mavlink_shim_stats_t packet;
	packet.x = x;
	packet.y = y;
	packet.vx = vx;
	packet.vy = vy;
	packet.A_proposed = A_proposed;
	packet.Theta_proposed = Theta_proposed;
	packet.a = a;
	packet.theta = theta;
	packet.throttle = throttle;
	packet.can_run1 = can_run1;
	packet.can_run2 = can_run2;
	packet.can_run3 = can_run3;
	packet.can_run4 = can_run4;
	packet.angle_boost = angle_boost;

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
 * @param can_run1 1 if shim 1 can run, 0 otherwise
 * @param can_run2 1 if shim 2 can run, 0 otherwise
 * @param can_run3 1 if shim 3 can run, 0 otherwise
 * @param can_run4 1 if shim 4 can run, 0 otherwise
 * @param x The value of x, relative to starting position.
 * @param y The height, relative to starting position.
 * @param vx The velocity in the x direction.
 * @param vy The velocity in the vertical direction.
 * @param throttle The proposed throttle received by the shim
 * @param angle_boost 1 if angle boost was requested, 0 otherwise
 * @param A_proposed The proposed acceleration, normal to the quadcopter.
 * @param Theta_proposed The proposed angle
 * @param a The issued acceleration, normal to the quadcopter.
 * @param theta The issued angle
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t mavlink_msg_shim_stats_pack_chan(uint8_t system_id, uint8_t component_id, uint8_t chan,
							   mavlink_message_t* msg,
						           uint8_t can_run1,uint8_t can_run2,uint8_t can_run3,uint8_t can_run4,float x,float y,float vx,float vy,uint16_t throttle,uint8_t angle_boost,float A_proposed,float Theta_proposed,float a,float theta)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
	char buf[MAVLINK_MSG_ID_SHIM_STATS_LEN];
	_mav_put_float(buf, 0, x);
	_mav_put_float(buf, 4, y);
	_mav_put_float(buf, 8, vx);
	_mav_put_float(buf, 12, vy);
	_mav_put_float(buf, 16, A_proposed);
	_mav_put_float(buf, 20, Theta_proposed);
	_mav_put_float(buf, 24, a);
	_mav_put_float(buf, 28, theta);
	_mav_put_uint16_t(buf, 32, throttle);
	_mav_put_uint8_t(buf, 34, can_run1);
	_mav_put_uint8_t(buf, 35, can_run2);
	_mav_put_uint8_t(buf, 36, can_run3);
	_mav_put_uint8_t(buf, 37, can_run4);
	_mav_put_uint8_t(buf, 38, angle_boost);

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), buf, MAVLINK_MSG_ID_SHIM_STATS_LEN);
#else
	mavlink_shim_stats_t packet;
	packet.x = x;
	packet.y = y;
	packet.vx = vx;
	packet.vy = vy;
	packet.A_proposed = A_proposed;
	packet.Theta_proposed = Theta_proposed;
	packet.a = a;
	packet.theta = theta;
	packet.throttle = throttle;
	packet.can_run1 = can_run1;
	packet.can_run2 = can_run2;
	packet.can_run3 = can_run3;
	packet.can_run4 = can_run4;
	packet.angle_boost = angle_boost;

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
	return mavlink_msg_shim_stats_pack(system_id, component_id, msg, shim_stats->can_run1, shim_stats->can_run2, shim_stats->can_run3, shim_stats->can_run4, shim_stats->x, shim_stats->y, shim_stats->vx, shim_stats->vy, shim_stats->throttle, shim_stats->angle_boost, shim_stats->A_proposed, shim_stats->Theta_proposed, shim_stats->a, shim_stats->theta);
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
	return mavlink_msg_shim_stats_pack_chan(system_id, component_id, chan, msg, shim_stats->can_run1, shim_stats->can_run2, shim_stats->can_run3, shim_stats->can_run4, shim_stats->x, shim_stats->y, shim_stats->vx, shim_stats->vy, shim_stats->throttle, shim_stats->angle_boost, shim_stats->A_proposed, shim_stats->Theta_proposed, shim_stats->a, shim_stats->theta);
}

/**
 * @brief Send a shim_stats message
 * @param chan MAVLink channel to send the message
 *
 * @param can_run1 1 if shim 1 can run, 0 otherwise
 * @param can_run2 1 if shim 2 can run, 0 otherwise
 * @param can_run3 1 if shim 3 can run, 0 otherwise
 * @param can_run4 1 if shim 4 can run, 0 otherwise
 * @param x The value of x, relative to starting position.
 * @param y The height, relative to starting position.
 * @param vx The velocity in the x direction.
 * @param vy The velocity in the vertical direction.
 * @param throttle The proposed throttle received by the shim
 * @param angle_boost 1 if angle boost was requested, 0 otherwise
 * @param A_proposed The proposed acceleration, normal to the quadcopter.
 * @param Theta_proposed The proposed angle
 * @param a The issued acceleration, normal to the quadcopter.
 * @param theta The issued angle
 */
#ifdef MAVLINK_USE_CONVENIENCE_FUNCTIONS

static inline void mavlink_msg_shim_stats_send(mavlink_channel_t chan, uint8_t can_run1, uint8_t can_run2, uint8_t can_run3, uint8_t can_run4, float x, float y, float vx, float vy, uint16_t throttle, uint8_t angle_boost, float A_proposed, float Theta_proposed, float a, float theta)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
	char buf[MAVLINK_MSG_ID_SHIM_STATS_LEN];
	_mav_put_float(buf, 0, x);
	_mav_put_float(buf, 4, y);
	_mav_put_float(buf, 8, vx);
	_mav_put_float(buf, 12, vy);
	_mav_put_float(buf, 16, A_proposed);
	_mav_put_float(buf, 20, Theta_proposed);
	_mav_put_float(buf, 24, a);
	_mav_put_float(buf, 28, theta);
	_mav_put_uint16_t(buf, 32, throttle);
	_mav_put_uint8_t(buf, 34, can_run1);
	_mav_put_uint8_t(buf, 35, can_run2);
	_mav_put_uint8_t(buf, 36, can_run3);
	_mav_put_uint8_t(buf, 37, can_run4);
	_mav_put_uint8_t(buf, 38, angle_boost);

#if MAVLINK_CRC_EXTRA
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_SHIM_STATS, buf, MAVLINK_MSG_ID_SHIM_STATS_LEN, MAVLINK_MSG_ID_SHIM_STATS_CRC);
#else
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_SHIM_STATS, buf, MAVLINK_MSG_ID_SHIM_STATS_LEN);
#endif
#else
	mavlink_shim_stats_t packet;
	packet.x = x;
	packet.y = y;
	packet.vx = vx;
	packet.vy = vy;
	packet.A_proposed = A_proposed;
	packet.Theta_proposed = Theta_proposed;
	packet.a = a;
	packet.theta = theta;
	packet.throttle = throttle;
	packet.can_run1 = can_run1;
	packet.can_run2 = can_run2;
	packet.can_run3 = can_run3;
	packet.can_run4 = can_run4;
	packet.angle_boost = angle_boost;

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
static inline void mavlink_msg_shim_stats_send_buf(mavlink_message_t *msgbuf, mavlink_channel_t chan,  uint8_t can_run1, uint8_t can_run2, uint8_t can_run3, uint8_t can_run4, float x, float y, float vx, float vy, uint16_t throttle, uint8_t angle_boost, float A_proposed, float Theta_proposed, float a, float theta)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
	char *buf = (char *)msgbuf;
	_mav_put_float(buf, 0, x);
	_mav_put_float(buf, 4, y);
	_mav_put_float(buf, 8, vx);
	_mav_put_float(buf, 12, vy);
	_mav_put_float(buf, 16, A_proposed);
	_mav_put_float(buf, 20, Theta_proposed);
	_mav_put_float(buf, 24, a);
	_mav_put_float(buf, 28, theta);
	_mav_put_uint16_t(buf, 32, throttle);
	_mav_put_uint8_t(buf, 34, can_run1);
	_mav_put_uint8_t(buf, 35, can_run2);
	_mav_put_uint8_t(buf, 36, can_run3);
	_mav_put_uint8_t(buf, 37, can_run4);
	_mav_put_uint8_t(buf, 38, angle_boost);

#if MAVLINK_CRC_EXTRA
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_SHIM_STATS, buf, MAVLINK_MSG_ID_SHIM_STATS_LEN, MAVLINK_MSG_ID_SHIM_STATS_CRC);
#else
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_SHIM_STATS, buf, MAVLINK_MSG_ID_SHIM_STATS_LEN);
#endif
#else
	mavlink_shim_stats_t *packet = (mavlink_shim_stats_t *)msgbuf;
	packet->x = x;
	packet->y = y;
	packet->vx = vx;
	packet->vy = vy;
	packet->A_proposed = A_proposed;
	packet->Theta_proposed = Theta_proposed;
	packet->a = a;
	packet->theta = theta;
	packet->throttle = throttle;
	packet->can_run1 = can_run1;
	packet->can_run2 = can_run2;
	packet->can_run3 = can_run3;
	packet->can_run4 = can_run4;
	packet->angle_boost = angle_boost;

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
 * @brief Get field can_run1 from shim_stats message
 *
 * @return 1 if shim 1 can run, 0 otherwise
 */
static inline uint8_t mavlink_msg_shim_stats_get_can_run1(const mavlink_message_t* msg)
{
	return _MAV_RETURN_uint8_t(msg,  34);
}

/**
 * @brief Get field can_run2 from shim_stats message
 *
 * @return 1 if shim 2 can run, 0 otherwise
 */
static inline uint8_t mavlink_msg_shim_stats_get_can_run2(const mavlink_message_t* msg)
{
	return _MAV_RETURN_uint8_t(msg,  35);
}

/**
 * @brief Get field can_run3 from shim_stats message
 *
 * @return 1 if shim 3 can run, 0 otherwise
 */
static inline uint8_t mavlink_msg_shim_stats_get_can_run3(const mavlink_message_t* msg)
{
	return _MAV_RETURN_uint8_t(msg,  36);
}

/**
 * @brief Get field can_run4 from shim_stats message
 *
 * @return 1 if shim 4 can run, 0 otherwise
 */
static inline uint8_t mavlink_msg_shim_stats_get_can_run4(const mavlink_message_t* msg)
{
	return _MAV_RETURN_uint8_t(msg,  37);
}

/**
 * @brief Get field x from shim_stats message
 *
 * @return The value of x, relative to starting position.
 */
static inline float mavlink_msg_shim_stats_get_x(const mavlink_message_t* msg)
{
	return _MAV_RETURN_float(msg,  0);
}

/**
 * @brief Get field y from shim_stats message
 *
 * @return The height, relative to starting position.
 */
static inline float mavlink_msg_shim_stats_get_y(const mavlink_message_t* msg)
{
	return _MAV_RETURN_float(msg,  4);
}

/**
 * @brief Get field vx from shim_stats message
 *
 * @return The velocity in the x direction.
 */
static inline float mavlink_msg_shim_stats_get_vx(const mavlink_message_t* msg)
{
	return _MAV_RETURN_float(msg,  8);
}

/**
 * @brief Get field vy from shim_stats message
 *
 * @return The velocity in the vertical direction.
 */
static inline float mavlink_msg_shim_stats_get_vy(const mavlink_message_t* msg)
{
	return _MAV_RETURN_float(msg,  12);
}

/**
 * @brief Get field throttle from shim_stats message
 *
 * @return The proposed throttle received by the shim
 */
static inline uint16_t mavlink_msg_shim_stats_get_throttle(const mavlink_message_t* msg)
{
	return _MAV_RETURN_uint16_t(msg,  32);
}

/**
 * @brief Get field angle_boost from shim_stats message
 *
 * @return 1 if angle boost was requested, 0 otherwise
 */
static inline uint8_t mavlink_msg_shim_stats_get_angle_boost(const mavlink_message_t* msg)
{
	return _MAV_RETURN_uint8_t(msg,  38);
}

/**
 * @brief Get field A_proposed from shim_stats message
 *
 * @return The proposed acceleration, normal to the quadcopter.
 */
static inline float mavlink_msg_shim_stats_get_A_proposed(const mavlink_message_t* msg)
{
	return _MAV_RETURN_float(msg,  16);
}

/**
 * @brief Get field Theta_proposed from shim_stats message
 *
 * @return The proposed angle
 */
static inline float mavlink_msg_shim_stats_get_Theta_proposed(const mavlink_message_t* msg)
{
	return _MAV_RETURN_float(msg,  20);
}

/**
 * @brief Get field a from shim_stats message
 *
 * @return The issued acceleration, normal to the quadcopter.
 */
static inline float mavlink_msg_shim_stats_get_a(const mavlink_message_t* msg)
{
	return _MAV_RETURN_float(msg,  24);
}

/**
 * @brief Get field theta from shim_stats message
 *
 * @return The issued angle
 */
static inline float mavlink_msg_shim_stats_get_theta(const mavlink_message_t* msg)
{
	return _MAV_RETURN_float(msg,  28);
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
	shim_stats->x = mavlink_msg_shim_stats_get_x(msg);
	shim_stats->y = mavlink_msg_shim_stats_get_y(msg);
	shim_stats->vx = mavlink_msg_shim_stats_get_vx(msg);
	shim_stats->vy = mavlink_msg_shim_stats_get_vy(msg);
	shim_stats->A_proposed = mavlink_msg_shim_stats_get_A_proposed(msg);
	shim_stats->Theta_proposed = mavlink_msg_shim_stats_get_Theta_proposed(msg);
	shim_stats->a = mavlink_msg_shim_stats_get_a(msg);
	shim_stats->theta = mavlink_msg_shim_stats_get_theta(msg);
	shim_stats->throttle = mavlink_msg_shim_stats_get_throttle(msg);
	shim_stats->can_run1 = mavlink_msg_shim_stats_get_can_run1(msg);
	shim_stats->can_run2 = mavlink_msg_shim_stats_get_can_run2(msg);
	shim_stats->can_run3 = mavlink_msg_shim_stats_get_can_run3(msg);
	shim_stats->can_run4 = mavlink_msg_shim_stats_get_can_run4(msg);
	shim_stats->angle_boost = mavlink_msg_shim_stats_get_angle_boost(msg);
#else
	memcpy(shim_stats, _MAV_PAYLOAD(msg), MAVLINK_MSG_ID_SHIM_STATS_LEN);
#endif
}
