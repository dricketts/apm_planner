// MESSAGE SHIM_STATS PACKING

#define MAVLINK_MSG_ID_SHIM_STATS 232

typedef struct __mavlink_shim_stats_t
{
 float x; ///< The value of x, relative to starting position.
 float y; ///< The height, relative to starting position.
 float vx; ///< The velocity in the x direction.
 float vy; ///< The velocity in the vertical direction.
 float A; ///< The proposed acceleration, normal to the quadcopter.
 float Theta; ///< The proposed angle
 float a; ///< The issued acceleration, normal to the quadcopter.
 float theta; ///< The issued angle
 float AX; ///< The proposed X acceleration.
 float AY; ///< The proposed Y acceleration.
 float ax; ///< The x acceleration.
 float ay; ///< The vertical acceleration.
 float amin_x; ///< The breaking acceleration for x.
 float amin_y; ///< The breaking acceleration for y.
 uint16_t throttle; ///< The proposed throttle received by the shim
 uint8_t something_weird; ///< 1 if something weird happened, 0 otherwise
 uint8_t angle_boost; ///< 1 if angle boost was requested, 0 otherwise
 uint8_t safe_x; ///< Whether or not the safety check passed for the X direction.
 uint8_t safe_y; ///< Whether or not the safety check passed for the Y direction.
 uint8_t safe_x_vel_ub; ///< Whether or not the safety check passed for the upper bound of velocity in the X direction.
 uint8_t safe_x_vel_lb; ///< Whether or not the safety check passed for the lower bound of velocity in the X direction.
 uint8_t safe_x_pos_ub; ///< Whether or not the safety check passed for the upper bound of position in the X direction.
 uint8_t safe_x_pos_lb; ///< Whether or not the safety check passed for the lower bound of position in the X direction.
 uint8_t safe_y_vel_ub; ///< Whether or not the safety check passed for the upper bound of velocity in the Y direction.
 uint8_t safe_y_vel_lb; ///< Whether or not the safety check passed for the lower bound of velocity in the Y direction.
 uint8_t safe_y_pos_ub; ///< Whether or not the safety check passed for the upper bound of position in the Y direction.
 uint8_t safe_y_pos_lb; ///< Whether or not the safety check passed for the lower bound of position in the Y direction.
 uint8_t Theta_bound_check; ///< Whether or not the safety check passed for the proposed angle.
} mavlink_shim_stats_t;

#define MAVLINK_MSG_ID_SHIM_STATS_LEN 71
#define MAVLINK_MSG_ID_232_LEN 71

#define MAVLINK_MSG_ID_SHIM_STATS_CRC 244
#define MAVLINK_MSG_ID_232_CRC 244



#define MAVLINK_MESSAGE_INFO_SHIM_STATS { \
	"SHIM_STATS", \
	28, \
	{  { "x", NULL, MAVLINK_TYPE_FLOAT, 0, 0, offsetof(mavlink_shim_stats_t, x) }, \
         { "y", NULL, MAVLINK_TYPE_FLOAT, 0, 4, offsetof(mavlink_shim_stats_t, y) }, \
         { "vx", NULL, MAVLINK_TYPE_FLOAT, 0, 8, offsetof(mavlink_shim_stats_t, vx) }, \
         { "vy", NULL, MAVLINK_TYPE_FLOAT, 0, 12, offsetof(mavlink_shim_stats_t, vy) }, \
         { "A", NULL, MAVLINK_TYPE_FLOAT, 0, 16, offsetof(mavlink_shim_stats_t, A) }, \
         { "Theta", NULL, MAVLINK_TYPE_FLOAT, 0, 20, offsetof(mavlink_shim_stats_t, Theta) }, \
         { "a", NULL, MAVLINK_TYPE_FLOAT, 0, 24, offsetof(mavlink_shim_stats_t, a) }, \
         { "theta", NULL, MAVLINK_TYPE_FLOAT, 0, 28, offsetof(mavlink_shim_stats_t, theta) }, \
         { "AX", NULL, MAVLINK_TYPE_FLOAT, 0, 32, offsetof(mavlink_shim_stats_t, AX) }, \
         { "AY", NULL, MAVLINK_TYPE_FLOAT, 0, 36, offsetof(mavlink_shim_stats_t, AY) }, \
         { "ax", NULL, MAVLINK_TYPE_FLOAT, 0, 40, offsetof(mavlink_shim_stats_t, ax) }, \
         { "ay", NULL, MAVLINK_TYPE_FLOAT, 0, 44, offsetof(mavlink_shim_stats_t, ay) }, \
         { "amin_x", NULL, MAVLINK_TYPE_FLOAT, 0, 48, offsetof(mavlink_shim_stats_t, amin_x) }, \
         { "amin_y", NULL, MAVLINK_TYPE_FLOAT, 0, 52, offsetof(mavlink_shim_stats_t, amin_y) }, \
         { "throttle", NULL, MAVLINK_TYPE_UINT16_T, 0, 56, offsetof(mavlink_shim_stats_t, throttle) }, \
         { "something_weird", NULL, MAVLINK_TYPE_UINT8_T, 0, 58, offsetof(mavlink_shim_stats_t, something_weird) }, \
         { "angle_boost", NULL, MAVLINK_TYPE_UINT8_T, 0, 59, offsetof(mavlink_shim_stats_t, angle_boost) }, \
         { "safe_x", NULL, MAVLINK_TYPE_UINT8_T, 0, 60, offsetof(mavlink_shim_stats_t, safe_x) }, \
         { "safe_y", NULL, MAVLINK_TYPE_UINT8_T, 0, 61, offsetof(mavlink_shim_stats_t, safe_y) }, \
         { "safe_x_vel_ub", NULL, MAVLINK_TYPE_UINT8_T, 0, 62, offsetof(mavlink_shim_stats_t, safe_x_vel_ub) }, \
         { "safe_x_vel_lb", NULL, MAVLINK_TYPE_UINT8_T, 0, 63, offsetof(mavlink_shim_stats_t, safe_x_vel_lb) }, \
         { "safe_x_pos_ub", NULL, MAVLINK_TYPE_UINT8_T, 0, 64, offsetof(mavlink_shim_stats_t, safe_x_pos_ub) }, \
         { "safe_x_pos_lb", NULL, MAVLINK_TYPE_UINT8_T, 0, 65, offsetof(mavlink_shim_stats_t, safe_x_pos_lb) }, \
         { "safe_y_vel_ub", NULL, MAVLINK_TYPE_UINT8_T, 0, 66, offsetof(mavlink_shim_stats_t, safe_y_vel_ub) }, \
         { "safe_y_vel_lb", NULL, MAVLINK_TYPE_UINT8_T, 0, 67, offsetof(mavlink_shim_stats_t, safe_y_vel_lb) }, \
         { "safe_y_pos_ub", NULL, MAVLINK_TYPE_UINT8_T, 0, 68, offsetof(mavlink_shim_stats_t, safe_y_pos_ub) }, \
         { "safe_y_pos_lb", NULL, MAVLINK_TYPE_UINT8_T, 0, 69, offsetof(mavlink_shim_stats_t, safe_y_pos_lb) }, \
         { "Theta_bound_check", NULL, MAVLINK_TYPE_UINT8_T, 0, 70, offsetof(mavlink_shim_stats_t, Theta_bound_check) }, \
         } \
}


/**
 * @brief Pack a shim_stats message
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param msg The MAVLink message to compress the data into
 *
 * @param something_weird 1 if something weird happened, 0 otherwise
 * @param x The value of x, relative to starting position.
 * @param y The height, relative to starting position.
 * @param vx The velocity in the x direction.
 * @param vy The velocity in the vertical direction.
 * @param throttle The proposed throttle received by the shim
 * @param angle_boost 1 if angle boost was requested, 0 otherwise
 * @param A The proposed acceleration, normal to the quadcopter.
 * @param Theta The proposed angle
 * @param a The issued acceleration, normal to the quadcopter.
 * @param theta The issued angle
 * @param AX The proposed X acceleration.
 * @param AY The proposed Y acceleration.
 * @param ax The x acceleration.
 * @param ay The vertical acceleration.
 * @param amin_x The breaking acceleration for x.
 * @param amin_y The breaking acceleration for y.
 * @param safe_x Whether or not the safety check passed for the X direction.
 * @param safe_y Whether or not the safety check passed for the Y direction.
 * @param safe_x_vel_ub Whether or not the safety check passed for the upper bound of velocity in the X direction.
 * @param safe_x_vel_lb Whether or not the safety check passed for the lower bound of velocity in the X direction.
 * @param safe_x_pos_ub Whether or not the safety check passed for the upper bound of position in the X direction.
 * @param safe_x_pos_lb Whether or not the safety check passed for the lower bound of position in the X direction.
 * @param safe_y_vel_ub Whether or not the safety check passed for the upper bound of velocity in the Y direction.
 * @param safe_y_vel_lb Whether or not the safety check passed for the lower bound of velocity in the Y direction.
 * @param safe_y_pos_ub Whether or not the safety check passed for the upper bound of position in the Y direction.
 * @param safe_y_pos_lb Whether or not the safety check passed for the lower bound of position in the Y direction.
 * @param Theta_bound_check Whether or not the safety check passed for the proposed angle.
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t mavlink_msg_shim_stats_pack(uint8_t system_id, uint8_t component_id, mavlink_message_t* msg,
						       uint8_t something_weird, float x, float y, float vx, float vy, uint16_t throttle, uint8_t angle_boost, float A, float Theta, float a, float theta, float AX, float AY, float ax, float ay, float amin_x, float amin_y, uint8_t safe_x, uint8_t safe_y, uint8_t safe_x_vel_ub, uint8_t safe_x_vel_lb, uint8_t safe_x_pos_ub, uint8_t safe_x_pos_lb, uint8_t safe_y_vel_ub, uint8_t safe_y_vel_lb, uint8_t safe_y_pos_ub, uint8_t safe_y_pos_lb, uint8_t Theta_bound_check)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
	char buf[MAVLINK_MSG_ID_SHIM_STATS_LEN];
	_mav_put_float(buf, 0, x);
	_mav_put_float(buf, 4, y);
	_mav_put_float(buf, 8, vx);
	_mav_put_float(buf, 12, vy);
	_mav_put_float(buf, 16, A);
	_mav_put_float(buf, 20, Theta);
	_mav_put_float(buf, 24, a);
	_mav_put_float(buf, 28, theta);
	_mav_put_float(buf, 32, AX);
	_mav_put_float(buf, 36, AY);
	_mav_put_float(buf, 40, ax);
	_mav_put_float(buf, 44, ay);
	_mav_put_float(buf, 48, amin_x);
	_mav_put_float(buf, 52, amin_y);
	_mav_put_uint16_t(buf, 56, throttle);
	_mav_put_uint8_t(buf, 58, something_weird);
	_mav_put_uint8_t(buf, 59, angle_boost);
	_mav_put_uint8_t(buf, 60, safe_x);
	_mav_put_uint8_t(buf, 61, safe_y);
	_mav_put_uint8_t(buf, 62, safe_x_vel_ub);
	_mav_put_uint8_t(buf, 63, safe_x_vel_lb);
	_mav_put_uint8_t(buf, 64, safe_x_pos_ub);
	_mav_put_uint8_t(buf, 65, safe_x_pos_lb);
	_mav_put_uint8_t(buf, 66, safe_y_vel_ub);
	_mav_put_uint8_t(buf, 67, safe_y_vel_lb);
	_mav_put_uint8_t(buf, 68, safe_y_pos_ub);
	_mav_put_uint8_t(buf, 69, safe_y_pos_lb);
	_mav_put_uint8_t(buf, 70, Theta_bound_check);

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), buf, MAVLINK_MSG_ID_SHIM_STATS_LEN);
#else
	mavlink_shim_stats_t packet;
	packet.x = x;
	packet.y = y;
	packet.vx = vx;
	packet.vy = vy;
	packet.A = A;
	packet.Theta = Theta;
	packet.a = a;
	packet.theta = theta;
	packet.AX = AX;
	packet.AY = AY;
	packet.ax = ax;
	packet.ay = ay;
	packet.amin_x = amin_x;
	packet.amin_y = amin_y;
	packet.throttle = throttle;
	packet.something_weird = something_weird;
	packet.angle_boost = angle_boost;
	packet.safe_x = safe_x;
	packet.safe_y = safe_y;
	packet.safe_x_vel_ub = safe_x_vel_ub;
	packet.safe_x_vel_lb = safe_x_vel_lb;
	packet.safe_x_pos_ub = safe_x_pos_ub;
	packet.safe_x_pos_lb = safe_x_pos_lb;
	packet.safe_y_vel_ub = safe_y_vel_ub;
	packet.safe_y_vel_lb = safe_y_vel_lb;
	packet.safe_y_pos_ub = safe_y_pos_ub;
	packet.safe_y_pos_lb = safe_y_pos_lb;
	packet.Theta_bound_check = Theta_bound_check;

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
 * @param something_weird 1 if something weird happened, 0 otherwise
 * @param x The value of x, relative to starting position.
 * @param y The height, relative to starting position.
 * @param vx The velocity in the x direction.
 * @param vy The velocity in the vertical direction.
 * @param throttle The proposed throttle received by the shim
 * @param angle_boost 1 if angle boost was requested, 0 otherwise
 * @param A The proposed acceleration, normal to the quadcopter.
 * @param Theta The proposed angle
 * @param a The issued acceleration, normal to the quadcopter.
 * @param theta The issued angle
 * @param AX The proposed X acceleration.
 * @param AY The proposed Y acceleration.
 * @param ax The x acceleration.
 * @param ay The vertical acceleration.
 * @param amin_x The breaking acceleration for x.
 * @param amin_y The breaking acceleration for y.
 * @param safe_x Whether or not the safety check passed for the X direction.
 * @param safe_y Whether or not the safety check passed for the Y direction.
 * @param safe_x_vel_ub Whether or not the safety check passed for the upper bound of velocity in the X direction.
 * @param safe_x_vel_lb Whether or not the safety check passed for the lower bound of velocity in the X direction.
 * @param safe_x_pos_ub Whether or not the safety check passed for the upper bound of position in the X direction.
 * @param safe_x_pos_lb Whether or not the safety check passed for the lower bound of position in the X direction.
 * @param safe_y_vel_ub Whether or not the safety check passed for the upper bound of velocity in the Y direction.
 * @param safe_y_vel_lb Whether or not the safety check passed for the lower bound of velocity in the Y direction.
 * @param safe_y_pos_ub Whether or not the safety check passed for the upper bound of position in the Y direction.
 * @param safe_y_pos_lb Whether or not the safety check passed for the lower bound of position in the Y direction.
 * @param Theta_bound_check Whether or not the safety check passed for the proposed angle.
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t mavlink_msg_shim_stats_pack_chan(uint8_t system_id, uint8_t component_id, uint8_t chan,
							   mavlink_message_t* msg,
						           uint8_t something_weird,float x,float y,float vx,float vy,uint16_t throttle,uint8_t angle_boost,float A,float Theta,float a,float theta,float AX,float AY,float ax,float ay,float amin_x,float amin_y,uint8_t safe_x,uint8_t safe_y,uint8_t safe_x_vel_ub,uint8_t safe_x_vel_lb,uint8_t safe_x_pos_ub,uint8_t safe_x_pos_lb,uint8_t safe_y_vel_ub,uint8_t safe_y_vel_lb,uint8_t safe_y_pos_ub,uint8_t safe_y_pos_lb,uint8_t Theta_bound_check)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
	char buf[MAVLINK_MSG_ID_SHIM_STATS_LEN];
	_mav_put_float(buf, 0, x);
	_mav_put_float(buf, 4, y);
	_mav_put_float(buf, 8, vx);
	_mav_put_float(buf, 12, vy);
	_mav_put_float(buf, 16, A);
	_mav_put_float(buf, 20, Theta);
	_mav_put_float(buf, 24, a);
	_mav_put_float(buf, 28, theta);
	_mav_put_float(buf, 32, AX);
	_mav_put_float(buf, 36, AY);
	_mav_put_float(buf, 40, ax);
	_mav_put_float(buf, 44, ay);
	_mav_put_float(buf, 48, amin_x);
	_mav_put_float(buf, 52, amin_y);
	_mav_put_uint16_t(buf, 56, throttle);
	_mav_put_uint8_t(buf, 58, something_weird);
	_mav_put_uint8_t(buf, 59, angle_boost);
	_mav_put_uint8_t(buf, 60, safe_x);
	_mav_put_uint8_t(buf, 61, safe_y);
	_mav_put_uint8_t(buf, 62, safe_x_vel_ub);
	_mav_put_uint8_t(buf, 63, safe_x_vel_lb);
	_mav_put_uint8_t(buf, 64, safe_x_pos_ub);
	_mav_put_uint8_t(buf, 65, safe_x_pos_lb);
	_mav_put_uint8_t(buf, 66, safe_y_vel_ub);
	_mav_put_uint8_t(buf, 67, safe_y_vel_lb);
	_mav_put_uint8_t(buf, 68, safe_y_pos_ub);
	_mav_put_uint8_t(buf, 69, safe_y_pos_lb);
	_mav_put_uint8_t(buf, 70, Theta_bound_check);

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), buf, MAVLINK_MSG_ID_SHIM_STATS_LEN);
#else
	mavlink_shim_stats_t packet;
	packet.x = x;
	packet.y = y;
	packet.vx = vx;
	packet.vy = vy;
	packet.A = A;
	packet.Theta = Theta;
	packet.a = a;
	packet.theta = theta;
	packet.AX = AX;
	packet.AY = AY;
	packet.ax = ax;
	packet.ay = ay;
	packet.amin_x = amin_x;
	packet.amin_y = amin_y;
	packet.throttle = throttle;
	packet.something_weird = something_weird;
	packet.angle_boost = angle_boost;
	packet.safe_x = safe_x;
	packet.safe_y = safe_y;
	packet.safe_x_vel_ub = safe_x_vel_ub;
	packet.safe_x_vel_lb = safe_x_vel_lb;
	packet.safe_x_pos_ub = safe_x_pos_ub;
	packet.safe_x_pos_lb = safe_x_pos_lb;
	packet.safe_y_vel_ub = safe_y_vel_ub;
	packet.safe_y_vel_lb = safe_y_vel_lb;
	packet.safe_y_pos_ub = safe_y_pos_ub;
	packet.safe_y_pos_lb = safe_y_pos_lb;
	packet.Theta_bound_check = Theta_bound_check;

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
	return mavlink_msg_shim_stats_pack(system_id, component_id, msg, shim_stats->something_weird, shim_stats->x, shim_stats->y, shim_stats->vx, shim_stats->vy, shim_stats->throttle, shim_stats->angle_boost, shim_stats->A, shim_stats->Theta, shim_stats->a, shim_stats->theta, shim_stats->AX, shim_stats->AY, shim_stats->ax, shim_stats->ay, shim_stats->amin_x, shim_stats->amin_y, shim_stats->safe_x, shim_stats->safe_y, shim_stats->safe_x_vel_ub, shim_stats->safe_x_vel_lb, shim_stats->safe_x_pos_ub, shim_stats->safe_x_pos_lb, shim_stats->safe_y_vel_ub, shim_stats->safe_y_vel_lb, shim_stats->safe_y_pos_ub, shim_stats->safe_y_pos_lb, shim_stats->Theta_bound_check);
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
	return mavlink_msg_shim_stats_pack_chan(system_id, component_id, chan, msg, shim_stats->something_weird, shim_stats->x, shim_stats->y, shim_stats->vx, shim_stats->vy, shim_stats->throttle, shim_stats->angle_boost, shim_stats->A, shim_stats->Theta, shim_stats->a, shim_stats->theta, shim_stats->AX, shim_stats->AY, shim_stats->ax, shim_stats->ay, shim_stats->amin_x, shim_stats->amin_y, shim_stats->safe_x, shim_stats->safe_y, shim_stats->safe_x_vel_ub, shim_stats->safe_x_vel_lb, shim_stats->safe_x_pos_ub, shim_stats->safe_x_pos_lb, shim_stats->safe_y_vel_ub, shim_stats->safe_y_vel_lb, shim_stats->safe_y_pos_ub, shim_stats->safe_y_pos_lb, shim_stats->Theta_bound_check);
}

/**
 * @brief Send a shim_stats message
 * @param chan MAVLink channel to send the message
 *
 * @param something_weird 1 if something weird happened, 0 otherwise
 * @param x The value of x, relative to starting position.
 * @param y The height, relative to starting position.
 * @param vx The velocity in the x direction.
 * @param vy The velocity in the vertical direction.
 * @param throttle The proposed throttle received by the shim
 * @param angle_boost 1 if angle boost was requested, 0 otherwise
 * @param A The proposed acceleration, normal to the quadcopter.
 * @param Theta The proposed angle
 * @param a The issued acceleration, normal to the quadcopter.
 * @param theta The issued angle
 * @param AX The proposed X acceleration.
 * @param AY The proposed Y acceleration.
 * @param ax The x acceleration.
 * @param ay The vertical acceleration.
 * @param amin_x The breaking acceleration for x.
 * @param amin_y The breaking acceleration for y.
 * @param safe_x Whether or not the safety check passed for the X direction.
 * @param safe_y Whether or not the safety check passed for the Y direction.
 * @param safe_x_vel_ub Whether or not the safety check passed for the upper bound of velocity in the X direction.
 * @param safe_x_vel_lb Whether or not the safety check passed for the lower bound of velocity in the X direction.
 * @param safe_x_pos_ub Whether or not the safety check passed for the upper bound of position in the X direction.
 * @param safe_x_pos_lb Whether or not the safety check passed for the lower bound of position in the X direction.
 * @param safe_y_vel_ub Whether or not the safety check passed for the upper bound of velocity in the Y direction.
 * @param safe_y_vel_lb Whether or not the safety check passed for the lower bound of velocity in the Y direction.
 * @param safe_y_pos_ub Whether or not the safety check passed for the upper bound of position in the Y direction.
 * @param safe_y_pos_lb Whether or not the safety check passed for the lower bound of position in the Y direction.
 * @param Theta_bound_check Whether or not the safety check passed for the proposed angle.
 */
#ifdef MAVLINK_USE_CONVENIENCE_FUNCTIONS

static inline void mavlink_msg_shim_stats_send(mavlink_channel_t chan, uint8_t something_weird, float x, float y, float vx, float vy, uint16_t throttle, uint8_t angle_boost, float A, float Theta, float a, float theta, float AX, float AY, float ax, float ay, float amin_x, float amin_y, uint8_t safe_x, uint8_t safe_y, uint8_t safe_x_vel_ub, uint8_t safe_x_vel_lb, uint8_t safe_x_pos_ub, uint8_t safe_x_pos_lb, uint8_t safe_y_vel_ub, uint8_t safe_y_vel_lb, uint8_t safe_y_pos_ub, uint8_t safe_y_pos_lb, uint8_t Theta_bound_check)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
	char buf[MAVLINK_MSG_ID_SHIM_STATS_LEN];
	_mav_put_float(buf, 0, x);
	_mav_put_float(buf, 4, y);
	_mav_put_float(buf, 8, vx);
	_mav_put_float(buf, 12, vy);
	_mav_put_float(buf, 16, A);
	_mav_put_float(buf, 20, Theta);
	_mav_put_float(buf, 24, a);
	_mav_put_float(buf, 28, theta);
	_mav_put_float(buf, 32, AX);
	_mav_put_float(buf, 36, AY);
	_mav_put_float(buf, 40, ax);
	_mav_put_float(buf, 44, ay);
	_mav_put_float(buf, 48, amin_x);
	_mav_put_float(buf, 52, amin_y);
	_mav_put_uint16_t(buf, 56, throttle);
	_mav_put_uint8_t(buf, 58, something_weird);
	_mav_put_uint8_t(buf, 59, angle_boost);
	_mav_put_uint8_t(buf, 60, safe_x);
	_mav_put_uint8_t(buf, 61, safe_y);
	_mav_put_uint8_t(buf, 62, safe_x_vel_ub);
	_mav_put_uint8_t(buf, 63, safe_x_vel_lb);
	_mav_put_uint8_t(buf, 64, safe_x_pos_ub);
	_mav_put_uint8_t(buf, 65, safe_x_pos_lb);
	_mav_put_uint8_t(buf, 66, safe_y_vel_ub);
	_mav_put_uint8_t(buf, 67, safe_y_vel_lb);
	_mav_put_uint8_t(buf, 68, safe_y_pos_ub);
	_mav_put_uint8_t(buf, 69, safe_y_pos_lb);
	_mav_put_uint8_t(buf, 70, Theta_bound_check);

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
	packet.A = A;
	packet.Theta = Theta;
	packet.a = a;
	packet.theta = theta;
	packet.AX = AX;
	packet.AY = AY;
	packet.ax = ax;
	packet.ay = ay;
	packet.amin_x = amin_x;
	packet.amin_y = amin_y;
	packet.throttle = throttle;
	packet.something_weird = something_weird;
	packet.angle_boost = angle_boost;
	packet.safe_x = safe_x;
	packet.safe_y = safe_y;
	packet.safe_x_vel_ub = safe_x_vel_ub;
	packet.safe_x_vel_lb = safe_x_vel_lb;
	packet.safe_x_pos_ub = safe_x_pos_ub;
	packet.safe_x_pos_lb = safe_x_pos_lb;
	packet.safe_y_vel_ub = safe_y_vel_ub;
	packet.safe_y_vel_lb = safe_y_vel_lb;
	packet.safe_y_pos_ub = safe_y_pos_ub;
	packet.safe_y_pos_lb = safe_y_pos_lb;
	packet.Theta_bound_check = Theta_bound_check;

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
static inline void mavlink_msg_shim_stats_send_buf(mavlink_message_t *msgbuf, mavlink_channel_t chan,  uint8_t something_weird, float x, float y, float vx, float vy, uint16_t throttle, uint8_t angle_boost, float A, float Theta, float a, float theta, float AX, float AY, float ax, float ay, float amin_x, float amin_y, uint8_t safe_x, uint8_t safe_y, uint8_t safe_x_vel_ub, uint8_t safe_x_vel_lb, uint8_t safe_x_pos_ub, uint8_t safe_x_pos_lb, uint8_t safe_y_vel_ub, uint8_t safe_y_vel_lb, uint8_t safe_y_pos_ub, uint8_t safe_y_pos_lb, uint8_t Theta_bound_check)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
	char *buf = (char *)msgbuf;
	_mav_put_float(buf, 0, x);
	_mav_put_float(buf, 4, y);
	_mav_put_float(buf, 8, vx);
	_mav_put_float(buf, 12, vy);
	_mav_put_float(buf, 16, A);
	_mav_put_float(buf, 20, Theta);
	_mav_put_float(buf, 24, a);
	_mav_put_float(buf, 28, theta);
	_mav_put_float(buf, 32, AX);
	_mav_put_float(buf, 36, AY);
	_mav_put_float(buf, 40, ax);
	_mav_put_float(buf, 44, ay);
	_mav_put_float(buf, 48, amin_x);
	_mav_put_float(buf, 52, amin_y);
	_mav_put_uint16_t(buf, 56, throttle);
	_mav_put_uint8_t(buf, 58, something_weird);
	_mav_put_uint8_t(buf, 59, angle_boost);
	_mav_put_uint8_t(buf, 60, safe_x);
	_mav_put_uint8_t(buf, 61, safe_y);
	_mav_put_uint8_t(buf, 62, safe_x_vel_ub);
	_mav_put_uint8_t(buf, 63, safe_x_vel_lb);
	_mav_put_uint8_t(buf, 64, safe_x_pos_ub);
	_mav_put_uint8_t(buf, 65, safe_x_pos_lb);
	_mav_put_uint8_t(buf, 66, safe_y_vel_ub);
	_mav_put_uint8_t(buf, 67, safe_y_vel_lb);
	_mav_put_uint8_t(buf, 68, safe_y_pos_ub);
	_mav_put_uint8_t(buf, 69, safe_y_pos_lb);
	_mav_put_uint8_t(buf, 70, Theta_bound_check);

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
	packet->A = A;
	packet->Theta = Theta;
	packet->a = a;
	packet->theta = theta;
	packet->AX = AX;
	packet->AY = AY;
	packet->ax = ax;
	packet->ay = ay;
	packet->amin_x = amin_x;
	packet->amin_y = amin_y;
	packet->throttle = throttle;
	packet->something_weird = something_weird;
	packet->angle_boost = angle_boost;
	packet->safe_x = safe_x;
	packet->safe_y = safe_y;
	packet->safe_x_vel_ub = safe_x_vel_ub;
	packet->safe_x_vel_lb = safe_x_vel_lb;
	packet->safe_x_pos_ub = safe_x_pos_ub;
	packet->safe_x_pos_lb = safe_x_pos_lb;
	packet->safe_y_vel_ub = safe_y_vel_ub;
	packet->safe_y_vel_lb = safe_y_vel_lb;
	packet->safe_y_pos_ub = safe_y_pos_ub;
	packet->safe_y_pos_lb = safe_y_pos_lb;
	packet->Theta_bound_check = Theta_bound_check;

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
 * @brief Get field something_weird from shim_stats message
 *
 * @return 1 if something weird happened, 0 otherwise
 */
static inline uint8_t mavlink_msg_shim_stats_get_something_weird(const mavlink_message_t* msg)
{
	return _MAV_RETURN_uint8_t(msg,  58);
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
	return _MAV_RETURN_uint16_t(msg,  56);
}

/**
 * @brief Get field angle_boost from shim_stats message
 *
 * @return 1 if angle boost was requested, 0 otherwise
 */
static inline uint8_t mavlink_msg_shim_stats_get_angle_boost(const mavlink_message_t* msg)
{
	return _MAV_RETURN_uint8_t(msg,  59);
}

/**
 * @brief Get field A from shim_stats message
 *
 * @return The proposed acceleration, normal to the quadcopter.
 */
static inline float mavlink_msg_shim_stats_get_A(const mavlink_message_t* msg)
{
	return _MAV_RETURN_float(msg,  16);
}

/**
 * @brief Get field Theta from shim_stats message
 *
 * @return The proposed angle
 */
static inline float mavlink_msg_shim_stats_get_Theta(const mavlink_message_t* msg)
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
 * @brief Get field AX from shim_stats message
 *
 * @return The proposed X acceleration.
 */
static inline float mavlink_msg_shim_stats_get_AX(const mavlink_message_t* msg)
{
	return _MAV_RETURN_float(msg,  32);
}

/**
 * @brief Get field AY from shim_stats message
 *
 * @return The proposed Y acceleration.
 */
static inline float mavlink_msg_shim_stats_get_AY(const mavlink_message_t* msg)
{
	return _MAV_RETURN_float(msg,  36);
}

/**
 * @brief Get field ax from shim_stats message
 *
 * @return The x acceleration.
 */
static inline float mavlink_msg_shim_stats_get_ax(const mavlink_message_t* msg)
{
	return _MAV_RETURN_float(msg,  40);
}

/**
 * @brief Get field ay from shim_stats message
 *
 * @return The vertical acceleration.
 */
static inline float mavlink_msg_shim_stats_get_ay(const mavlink_message_t* msg)
{
	return _MAV_RETURN_float(msg,  44);
}

/**
 * @brief Get field amin_x from shim_stats message
 *
 * @return The breaking acceleration for x.
 */
static inline float mavlink_msg_shim_stats_get_amin_x(const mavlink_message_t* msg)
{
	return _MAV_RETURN_float(msg,  48);
}

/**
 * @brief Get field amin_y from shim_stats message
 *
 * @return The breaking acceleration for y.
 */
static inline float mavlink_msg_shim_stats_get_amin_y(const mavlink_message_t* msg)
{
	return _MAV_RETURN_float(msg,  52);
}

/**
 * @brief Get field safe_x from shim_stats message
 *
 * @return Whether or not the safety check passed for the X direction.
 */
static inline uint8_t mavlink_msg_shim_stats_get_safe_x(const mavlink_message_t* msg)
{
	return _MAV_RETURN_uint8_t(msg,  60);
}

/**
 * @brief Get field safe_y from shim_stats message
 *
 * @return Whether or not the safety check passed for the Y direction.
 */
static inline uint8_t mavlink_msg_shim_stats_get_safe_y(const mavlink_message_t* msg)
{
	return _MAV_RETURN_uint8_t(msg,  61);
}

/**
 * @brief Get field safe_x_vel_ub from shim_stats message
 *
 * @return Whether or not the safety check passed for the upper bound of velocity in the X direction.
 */
static inline uint8_t mavlink_msg_shim_stats_get_safe_x_vel_ub(const mavlink_message_t* msg)
{
	return _MAV_RETURN_uint8_t(msg,  62);
}

/**
 * @brief Get field safe_x_vel_lb from shim_stats message
 *
 * @return Whether or not the safety check passed for the lower bound of velocity in the X direction.
 */
static inline uint8_t mavlink_msg_shim_stats_get_safe_x_vel_lb(const mavlink_message_t* msg)
{
	return _MAV_RETURN_uint8_t(msg,  63);
}

/**
 * @brief Get field safe_x_pos_ub from shim_stats message
 *
 * @return Whether or not the safety check passed for the upper bound of position in the X direction.
 */
static inline uint8_t mavlink_msg_shim_stats_get_safe_x_pos_ub(const mavlink_message_t* msg)
{
	return _MAV_RETURN_uint8_t(msg,  64);
}

/**
 * @brief Get field safe_x_pos_lb from shim_stats message
 *
 * @return Whether or not the safety check passed for the lower bound of position in the X direction.
 */
static inline uint8_t mavlink_msg_shim_stats_get_safe_x_pos_lb(const mavlink_message_t* msg)
{
	return _MAV_RETURN_uint8_t(msg,  65);
}

/**
 * @brief Get field safe_y_vel_ub from shim_stats message
 *
 * @return Whether or not the safety check passed for the upper bound of velocity in the Y direction.
 */
static inline uint8_t mavlink_msg_shim_stats_get_safe_y_vel_ub(const mavlink_message_t* msg)
{
	return _MAV_RETURN_uint8_t(msg,  66);
}

/**
 * @brief Get field safe_y_vel_lb from shim_stats message
 *
 * @return Whether or not the safety check passed for the lower bound of velocity in the Y direction.
 */
static inline uint8_t mavlink_msg_shim_stats_get_safe_y_vel_lb(const mavlink_message_t* msg)
{
	return _MAV_RETURN_uint8_t(msg,  67);
}

/**
 * @brief Get field safe_y_pos_ub from shim_stats message
 *
 * @return Whether or not the safety check passed for the upper bound of position in the Y direction.
 */
static inline uint8_t mavlink_msg_shim_stats_get_safe_y_pos_ub(const mavlink_message_t* msg)
{
	return _MAV_RETURN_uint8_t(msg,  68);
}

/**
 * @brief Get field safe_y_pos_lb from shim_stats message
 *
 * @return Whether or not the safety check passed for the lower bound of position in the Y direction.
 */
static inline uint8_t mavlink_msg_shim_stats_get_safe_y_pos_lb(const mavlink_message_t* msg)
{
	return _MAV_RETURN_uint8_t(msg,  69);
}

/**
 * @brief Get field Theta_bound_check from shim_stats message
 *
 * @return Whether or not the safety check passed for the proposed angle.
 */
static inline uint8_t mavlink_msg_shim_stats_get_Theta_bound_check(const mavlink_message_t* msg)
{
	return _MAV_RETURN_uint8_t(msg,  70);
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
	shim_stats->A = mavlink_msg_shim_stats_get_A(msg);
	shim_stats->Theta = mavlink_msg_shim_stats_get_Theta(msg);
	shim_stats->a = mavlink_msg_shim_stats_get_a(msg);
	shim_stats->theta = mavlink_msg_shim_stats_get_theta(msg);
	shim_stats->AX = mavlink_msg_shim_stats_get_AX(msg);
	shim_stats->AY = mavlink_msg_shim_stats_get_AY(msg);
	shim_stats->ax = mavlink_msg_shim_stats_get_ax(msg);
	shim_stats->ay = mavlink_msg_shim_stats_get_ay(msg);
	shim_stats->amin_x = mavlink_msg_shim_stats_get_amin_x(msg);
	shim_stats->amin_y = mavlink_msg_shim_stats_get_amin_y(msg);
	shim_stats->throttle = mavlink_msg_shim_stats_get_throttle(msg);
	shim_stats->something_weird = mavlink_msg_shim_stats_get_something_weird(msg);
	shim_stats->angle_boost = mavlink_msg_shim_stats_get_angle_boost(msg);
	shim_stats->safe_x = mavlink_msg_shim_stats_get_safe_x(msg);
	shim_stats->safe_y = mavlink_msg_shim_stats_get_safe_y(msg);
	shim_stats->safe_x_vel_ub = mavlink_msg_shim_stats_get_safe_x_vel_ub(msg);
	shim_stats->safe_x_vel_lb = mavlink_msg_shim_stats_get_safe_x_vel_lb(msg);
	shim_stats->safe_x_pos_ub = mavlink_msg_shim_stats_get_safe_x_pos_ub(msg);
	shim_stats->safe_x_pos_lb = mavlink_msg_shim_stats_get_safe_x_pos_lb(msg);
	shim_stats->safe_y_vel_ub = mavlink_msg_shim_stats_get_safe_y_vel_ub(msg);
	shim_stats->safe_y_vel_lb = mavlink_msg_shim_stats_get_safe_y_vel_lb(msg);
	shim_stats->safe_y_pos_ub = mavlink_msg_shim_stats_get_safe_y_pos_ub(msg);
	shim_stats->safe_y_pos_lb = mavlink_msg_shim_stats_get_safe_y_pos_lb(msg);
	shim_stats->Theta_bound_check = mavlink_msg_shim_stats_get_Theta_bound_check(msg);
#else
	memcpy(shim_stats, _MAV_PAYLOAD(msg), MAVLINK_MSG_ID_SHIM_STATS_LEN);
#endif
}
