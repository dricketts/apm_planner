// MESSAGE SHIM_PARAMS PACKING

#define MAVLINK_MSG_ID_SHIM_PARAMS 231

typedef struct __mavlink_shim_params_t
{
 float d_ctrl; ///< 1 to enable smoothing, 0 to disable
 float roll_lb; ///< the lower bound on roll angle allowed by the shim in (??DEGREES??)
 int16_t abraking; ///< The braking acceleration in cm/s/s issued when the height-dimension shim is engaged. Should be at least the acceleration due to gravity.
 uint16_t mid_throttle; ///< The throttle required for hover.
 int16_t y_ub1; ///< the height upper bound of the shim in cm
 int16_t y_lb1; ///< the height lower bound of the shim in cm
 int16_t vy_ub1; ///< the vertical velocity upper bound of the shim in cm/s
 int16_t x_ub1; ///< the upper bound for the "x" dimension of the shim in cm
 int16_t x_lb1; ///< the lower bound for the "x" dimension of the shim in cm
 int16_t vx_ub1; ///< the "x" dimension velocity upper bound of the shim in cm/s
 int16_t y_ub2; ///< the height upper bound of the shim in cm
 int16_t y_lb2; ///< the height lower bound of the shim in cm
 int16_t vy_ub2; ///< the vertical velocity upper bound of the shim in cm/s
 int16_t x_ub2; ///< the upper bound for the "x" dimension of the shim in cm
 int16_t x_lb2; ///< the lower bound for the "x" dimension of the shim in cm
 int16_t vx_ub2; ///< the "x" dimension velocity upper bound of the shim in cm/s
 int16_t y_ub3; ///< the height upper bound of the shim in cm
 int16_t y_lb3; ///< the height lower bound of the shim in cm
 int16_t vy_ub3; ///< the vertical velocity upper bound of the shim in cm/s
 int16_t x_ub3; ///< the upper bound for the "x" dimension of the shim in cm
 int16_t x_lb3; ///< the lower bound for the "x" dimension of the shim in cm
 int16_t vx_ub3; ///< the "x" dimension velocity upper bound of the shim in cm/s
 int16_t y_ub4; ///< the height upper bound of the shim in cm
 int16_t y_lb4; ///< the height lower bound of the shim in cm
 int16_t vy_ub4; ///< the vertical velocity upper bound of the shim in cm/s
 int16_t x_ub4; ///< the upper bound for the "x" dimension of the shim in cm
 int16_t x_lb4; ///< the lower bound for the "x" dimension of the shim in cm
 int16_t vx_ub4; ///< the "x" dimension velocity upper bound of the shim in cm/s
 uint8_t smooth; ///< 1 to enable smoothing, 0 to disable
 uint8_t lookahead; ///< lookahead factor for the smoothing shim
} mavlink_shim_params_t;

#define MAVLINK_MSG_ID_SHIM_PARAMS_LEN 62
#define MAVLINK_MSG_ID_231_LEN 62

#define MAVLINK_MSG_ID_SHIM_PARAMS_CRC 162
#define MAVLINK_MSG_ID_231_CRC 162



#define MAVLINK_MESSAGE_INFO_SHIM_PARAMS { \
	"SHIM_PARAMS", \
	30, \
	{  { "d_ctrl", NULL, MAVLINK_TYPE_FLOAT, 0, 0, offsetof(mavlink_shim_params_t, d_ctrl) }, \
         { "roll_lb", NULL, MAVLINK_TYPE_FLOAT, 0, 4, offsetof(mavlink_shim_params_t, roll_lb) }, \
         { "abraking", NULL, MAVLINK_TYPE_INT16_T, 0, 8, offsetof(mavlink_shim_params_t, abraking) }, \
         { "mid_throttle", NULL, MAVLINK_TYPE_UINT16_T, 0, 10, offsetof(mavlink_shim_params_t, mid_throttle) }, \
         { "y_ub1", NULL, MAVLINK_TYPE_INT16_T, 0, 12, offsetof(mavlink_shim_params_t, y_ub1) }, \
         { "y_lb1", NULL, MAVLINK_TYPE_INT16_T, 0, 14, offsetof(mavlink_shim_params_t, y_lb1) }, \
         { "vy_ub1", NULL, MAVLINK_TYPE_INT16_T, 0, 16, offsetof(mavlink_shim_params_t, vy_ub1) }, \
         { "x_ub1", NULL, MAVLINK_TYPE_INT16_T, 0, 18, offsetof(mavlink_shim_params_t, x_ub1) }, \
         { "x_lb1", NULL, MAVLINK_TYPE_INT16_T, 0, 20, offsetof(mavlink_shim_params_t, x_lb1) }, \
         { "vx_ub1", NULL, MAVLINK_TYPE_INT16_T, 0, 22, offsetof(mavlink_shim_params_t, vx_ub1) }, \
         { "y_ub2", NULL, MAVLINK_TYPE_INT16_T, 0, 24, offsetof(mavlink_shim_params_t, y_ub2) }, \
         { "y_lb2", NULL, MAVLINK_TYPE_INT16_T, 0, 26, offsetof(mavlink_shim_params_t, y_lb2) }, \
         { "vy_ub2", NULL, MAVLINK_TYPE_INT16_T, 0, 28, offsetof(mavlink_shim_params_t, vy_ub2) }, \
         { "x_ub2", NULL, MAVLINK_TYPE_INT16_T, 0, 30, offsetof(mavlink_shim_params_t, x_ub2) }, \
         { "x_lb2", NULL, MAVLINK_TYPE_INT16_T, 0, 32, offsetof(mavlink_shim_params_t, x_lb2) }, \
         { "vx_ub2", NULL, MAVLINK_TYPE_INT16_T, 0, 34, offsetof(mavlink_shim_params_t, vx_ub2) }, \
         { "y_ub3", NULL, MAVLINK_TYPE_INT16_T, 0, 36, offsetof(mavlink_shim_params_t, y_ub3) }, \
         { "y_lb3", NULL, MAVLINK_TYPE_INT16_T, 0, 38, offsetof(mavlink_shim_params_t, y_lb3) }, \
         { "vy_ub3", NULL, MAVLINK_TYPE_INT16_T, 0, 40, offsetof(mavlink_shim_params_t, vy_ub3) }, \
         { "x_ub3", NULL, MAVLINK_TYPE_INT16_T, 0, 42, offsetof(mavlink_shim_params_t, x_ub3) }, \
         { "x_lb3", NULL, MAVLINK_TYPE_INT16_T, 0, 44, offsetof(mavlink_shim_params_t, x_lb3) }, \
         { "vx_ub3", NULL, MAVLINK_TYPE_INT16_T, 0, 46, offsetof(mavlink_shim_params_t, vx_ub3) }, \
         { "y_ub4", NULL, MAVLINK_TYPE_INT16_T, 0, 48, offsetof(mavlink_shim_params_t, y_ub4) }, \
         { "y_lb4", NULL, MAVLINK_TYPE_INT16_T, 0, 50, offsetof(mavlink_shim_params_t, y_lb4) }, \
         { "vy_ub4", NULL, MAVLINK_TYPE_INT16_T, 0, 52, offsetof(mavlink_shim_params_t, vy_ub4) }, \
         { "x_ub4", NULL, MAVLINK_TYPE_INT16_T, 0, 54, offsetof(mavlink_shim_params_t, x_ub4) }, \
         { "x_lb4", NULL, MAVLINK_TYPE_INT16_T, 0, 56, offsetof(mavlink_shim_params_t, x_lb4) }, \
         { "vx_ub4", NULL, MAVLINK_TYPE_INT16_T, 0, 58, offsetof(mavlink_shim_params_t, vx_ub4) }, \
         { "smooth", NULL, MAVLINK_TYPE_UINT8_T, 0, 60, offsetof(mavlink_shim_params_t, smooth) }, \
         { "lookahead", NULL, MAVLINK_TYPE_UINT8_T, 0, 61, offsetof(mavlink_shim_params_t, lookahead) }, \
         } \
}


/**
 * @brief Pack a shim_params message
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param msg The MAVLink message to compress the data into
 *
 * @param d_ctrl 1 to enable smoothing, 0 to disable
 * @param smooth 1 to enable smoothing, 0 to disable
 * @param lookahead lookahead factor for the smoothing shim
 * @param roll_lb the lower bound on roll angle allowed by the shim in (??DEGREES??)
 * @param abraking The braking acceleration in cm/s/s issued when the height-dimension shim is engaged. Should be at least the acceleration due to gravity.
 * @param mid_throttle The throttle required for hover.
 * @param y_ub1 the height upper bound of the shim in cm
 * @param y_lb1 the height lower bound of the shim in cm
 * @param vy_ub1 the vertical velocity upper bound of the shim in cm/s
 * @param x_ub1 the upper bound for the "x" dimension of the shim in cm
 * @param x_lb1 the lower bound for the "x" dimension of the shim in cm
 * @param vx_ub1 the "x" dimension velocity upper bound of the shim in cm/s
 * @param y_ub2 the height upper bound of the shim in cm
 * @param y_lb2 the height lower bound of the shim in cm
 * @param vy_ub2 the vertical velocity upper bound of the shim in cm/s
 * @param x_ub2 the upper bound for the "x" dimension of the shim in cm
 * @param x_lb2 the lower bound for the "x" dimension of the shim in cm
 * @param vx_ub2 the "x" dimension velocity upper bound of the shim in cm/s
 * @param y_ub3 the height upper bound of the shim in cm
 * @param y_lb3 the height lower bound of the shim in cm
 * @param vy_ub3 the vertical velocity upper bound of the shim in cm/s
 * @param x_ub3 the upper bound for the "x" dimension of the shim in cm
 * @param x_lb3 the lower bound for the "x" dimension of the shim in cm
 * @param vx_ub3 the "x" dimension velocity upper bound of the shim in cm/s
 * @param y_ub4 the height upper bound of the shim in cm
 * @param y_lb4 the height lower bound of the shim in cm
 * @param vy_ub4 the vertical velocity upper bound of the shim in cm/s
 * @param x_ub4 the upper bound for the "x" dimension of the shim in cm
 * @param x_lb4 the lower bound for the "x" dimension of the shim in cm
 * @param vx_ub4 the "x" dimension velocity upper bound of the shim in cm/s
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t mavlink_msg_shim_params_pack(uint8_t system_id, uint8_t component_id, mavlink_message_t* msg,
						       float d_ctrl, uint8_t smooth, uint8_t lookahead, float roll_lb, int16_t abraking, uint16_t mid_throttle, int16_t y_ub1, int16_t y_lb1, int16_t vy_ub1, int16_t x_ub1, int16_t x_lb1, int16_t vx_ub1, int16_t y_ub2, int16_t y_lb2, int16_t vy_ub2, int16_t x_ub2, int16_t x_lb2, int16_t vx_ub2, int16_t y_ub3, int16_t y_lb3, int16_t vy_ub3, int16_t x_ub3, int16_t x_lb3, int16_t vx_ub3, int16_t y_ub4, int16_t y_lb4, int16_t vy_ub4, int16_t x_ub4, int16_t x_lb4, int16_t vx_ub4)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
	char buf[MAVLINK_MSG_ID_SHIM_PARAMS_LEN];
	_mav_put_float(buf, 0, d_ctrl);
	_mav_put_float(buf, 4, roll_lb);
	_mav_put_int16_t(buf, 8, abraking);
	_mav_put_uint16_t(buf, 10, mid_throttle);
	_mav_put_int16_t(buf, 12, y_ub1);
	_mav_put_int16_t(buf, 14, y_lb1);
	_mav_put_int16_t(buf, 16, vy_ub1);
	_mav_put_int16_t(buf, 18, x_ub1);
	_mav_put_int16_t(buf, 20, x_lb1);
	_mav_put_int16_t(buf, 22, vx_ub1);
	_mav_put_int16_t(buf, 24, y_ub2);
	_mav_put_int16_t(buf, 26, y_lb2);
	_mav_put_int16_t(buf, 28, vy_ub2);
	_mav_put_int16_t(buf, 30, x_ub2);
	_mav_put_int16_t(buf, 32, x_lb2);
	_mav_put_int16_t(buf, 34, vx_ub2);
	_mav_put_int16_t(buf, 36, y_ub3);
	_mav_put_int16_t(buf, 38, y_lb3);
	_mav_put_int16_t(buf, 40, vy_ub3);
	_mav_put_int16_t(buf, 42, x_ub3);
	_mav_put_int16_t(buf, 44, x_lb3);
	_mav_put_int16_t(buf, 46, vx_ub3);
	_mav_put_int16_t(buf, 48, y_ub4);
	_mav_put_int16_t(buf, 50, y_lb4);
	_mav_put_int16_t(buf, 52, vy_ub4);
	_mav_put_int16_t(buf, 54, x_ub4);
	_mav_put_int16_t(buf, 56, x_lb4);
	_mav_put_int16_t(buf, 58, vx_ub4);
	_mav_put_uint8_t(buf, 60, smooth);
	_mav_put_uint8_t(buf, 61, lookahead);

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), buf, MAVLINK_MSG_ID_SHIM_PARAMS_LEN);
#else
	mavlink_shim_params_t packet;
	packet.d_ctrl = d_ctrl;
	packet.roll_lb = roll_lb;
	packet.abraking = abraking;
	packet.mid_throttle = mid_throttle;
	packet.y_ub1 = y_ub1;
	packet.y_lb1 = y_lb1;
	packet.vy_ub1 = vy_ub1;
	packet.x_ub1 = x_ub1;
	packet.x_lb1 = x_lb1;
	packet.vx_ub1 = vx_ub1;
	packet.y_ub2 = y_ub2;
	packet.y_lb2 = y_lb2;
	packet.vy_ub2 = vy_ub2;
	packet.x_ub2 = x_ub2;
	packet.x_lb2 = x_lb2;
	packet.vx_ub2 = vx_ub2;
	packet.y_ub3 = y_ub3;
	packet.y_lb3 = y_lb3;
	packet.vy_ub3 = vy_ub3;
	packet.x_ub3 = x_ub3;
	packet.x_lb3 = x_lb3;
	packet.vx_ub3 = vx_ub3;
	packet.y_ub4 = y_ub4;
	packet.y_lb4 = y_lb4;
	packet.vy_ub4 = vy_ub4;
	packet.x_ub4 = x_ub4;
	packet.x_lb4 = x_lb4;
	packet.vx_ub4 = vx_ub4;
	packet.smooth = smooth;
	packet.lookahead = lookahead;

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
 * @param d_ctrl 1 to enable smoothing, 0 to disable
 * @param smooth 1 to enable smoothing, 0 to disable
 * @param lookahead lookahead factor for the smoothing shim
 * @param roll_lb the lower bound on roll angle allowed by the shim in (??DEGREES??)
 * @param abraking The braking acceleration in cm/s/s issued when the height-dimension shim is engaged. Should be at least the acceleration due to gravity.
 * @param mid_throttle The throttle required for hover.
 * @param y_ub1 the height upper bound of the shim in cm
 * @param y_lb1 the height lower bound of the shim in cm
 * @param vy_ub1 the vertical velocity upper bound of the shim in cm/s
 * @param x_ub1 the upper bound for the "x" dimension of the shim in cm
 * @param x_lb1 the lower bound for the "x" dimension of the shim in cm
 * @param vx_ub1 the "x" dimension velocity upper bound of the shim in cm/s
 * @param y_ub2 the height upper bound of the shim in cm
 * @param y_lb2 the height lower bound of the shim in cm
 * @param vy_ub2 the vertical velocity upper bound of the shim in cm/s
 * @param x_ub2 the upper bound for the "x" dimension of the shim in cm
 * @param x_lb2 the lower bound for the "x" dimension of the shim in cm
 * @param vx_ub2 the "x" dimension velocity upper bound of the shim in cm/s
 * @param y_ub3 the height upper bound of the shim in cm
 * @param y_lb3 the height lower bound of the shim in cm
 * @param vy_ub3 the vertical velocity upper bound of the shim in cm/s
 * @param x_ub3 the upper bound for the "x" dimension of the shim in cm
 * @param x_lb3 the lower bound for the "x" dimension of the shim in cm
 * @param vx_ub3 the "x" dimension velocity upper bound of the shim in cm/s
 * @param y_ub4 the height upper bound of the shim in cm
 * @param y_lb4 the height lower bound of the shim in cm
 * @param vy_ub4 the vertical velocity upper bound of the shim in cm/s
 * @param x_ub4 the upper bound for the "x" dimension of the shim in cm
 * @param x_lb4 the lower bound for the "x" dimension of the shim in cm
 * @param vx_ub4 the "x" dimension velocity upper bound of the shim in cm/s
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t mavlink_msg_shim_params_pack_chan(uint8_t system_id, uint8_t component_id, uint8_t chan,
							   mavlink_message_t* msg,
						           float d_ctrl,uint8_t smooth,uint8_t lookahead,float roll_lb,int16_t abraking,uint16_t mid_throttle,int16_t y_ub1,int16_t y_lb1,int16_t vy_ub1,int16_t x_ub1,int16_t x_lb1,int16_t vx_ub1,int16_t y_ub2,int16_t y_lb2,int16_t vy_ub2,int16_t x_ub2,int16_t x_lb2,int16_t vx_ub2,int16_t y_ub3,int16_t y_lb3,int16_t vy_ub3,int16_t x_ub3,int16_t x_lb3,int16_t vx_ub3,int16_t y_ub4,int16_t y_lb4,int16_t vy_ub4,int16_t x_ub4,int16_t x_lb4,int16_t vx_ub4)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
	char buf[MAVLINK_MSG_ID_SHIM_PARAMS_LEN];
	_mav_put_float(buf, 0, d_ctrl);
	_mav_put_float(buf, 4, roll_lb);
	_mav_put_int16_t(buf, 8, abraking);
	_mav_put_uint16_t(buf, 10, mid_throttle);
	_mav_put_int16_t(buf, 12, y_ub1);
	_mav_put_int16_t(buf, 14, y_lb1);
	_mav_put_int16_t(buf, 16, vy_ub1);
	_mav_put_int16_t(buf, 18, x_ub1);
	_mav_put_int16_t(buf, 20, x_lb1);
	_mav_put_int16_t(buf, 22, vx_ub1);
	_mav_put_int16_t(buf, 24, y_ub2);
	_mav_put_int16_t(buf, 26, y_lb2);
	_mav_put_int16_t(buf, 28, vy_ub2);
	_mav_put_int16_t(buf, 30, x_ub2);
	_mav_put_int16_t(buf, 32, x_lb2);
	_mav_put_int16_t(buf, 34, vx_ub2);
	_mav_put_int16_t(buf, 36, y_ub3);
	_mav_put_int16_t(buf, 38, y_lb3);
	_mav_put_int16_t(buf, 40, vy_ub3);
	_mav_put_int16_t(buf, 42, x_ub3);
	_mav_put_int16_t(buf, 44, x_lb3);
	_mav_put_int16_t(buf, 46, vx_ub3);
	_mav_put_int16_t(buf, 48, y_ub4);
	_mav_put_int16_t(buf, 50, y_lb4);
	_mav_put_int16_t(buf, 52, vy_ub4);
	_mav_put_int16_t(buf, 54, x_ub4);
	_mav_put_int16_t(buf, 56, x_lb4);
	_mav_put_int16_t(buf, 58, vx_ub4);
	_mav_put_uint8_t(buf, 60, smooth);
	_mav_put_uint8_t(buf, 61, lookahead);

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), buf, MAVLINK_MSG_ID_SHIM_PARAMS_LEN);
#else
	mavlink_shim_params_t packet;
	packet.d_ctrl = d_ctrl;
	packet.roll_lb = roll_lb;
	packet.abraking = abraking;
	packet.mid_throttle = mid_throttle;
	packet.y_ub1 = y_ub1;
	packet.y_lb1 = y_lb1;
	packet.vy_ub1 = vy_ub1;
	packet.x_ub1 = x_ub1;
	packet.x_lb1 = x_lb1;
	packet.vx_ub1 = vx_ub1;
	packet.y_ub2 = y_ub2;
	packet.y_lb2 = y_lb2;
	packet.vy_ub2 = vy_ub2;
	packet.x_ub2 = x_ub2;
	packet.x_lb2 = x_lb2;
	packet.vx_ub2 = vx_ub2;
	packet.y_ub3 = y_ub3;
	packet.y_lb3 = y_lb3;
	packet.vy_ub3 = vy_ub3;
	packet.x_ub3 = x_ub3;
	packet.x_lb3 = x_lb3;
	packet.vx_ub3 = vx_ub3;
	packet.y_ub4 = y_ub4;
	packet.y_lb4 = y_lb4;
	packet.vy_ub4 = vy_ub4;
	packet.x_ub4 = x_ub4;
	packet.x_lb4 = x_lb4;
	packet.vx_ub4 = vx_ub4;
	packet.smooth = smooth;
	packet.lookahead = lookahead;

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
	return mavlink_msg_shim_params_pack(system_id, component_id, msg, shim_params->d_ctrl, shim_params->smooth, shim_params->lookahead, shim_params->roll_lb, shim_params->abraking, shim_params->mid_throttle, shim_params->y_ub1, shim_params->y_lb1, shim_params->vy_ub1, shim_params->x_ub1, shim_params->x_lb1, shim_params->vx_ub1, shim_params->y_ub2, shim_params->y_lb2, shim_params->vy_ub2, shim_params->x_ub2, shim_params->x_lb2, shim_params->vx_ub2, shim_params->y_ub3, shim_params->y_lb3, shim_params->vy_ub3, shim_params->x_ub3, shim_params->x_lb3, shim_params->vx_ub3, shim_params->y_ub4, shim_params->y_lb4, shim_params->vy_ub4, shim_params->x_ub4, shim_params->x_lb4, shim_params->vx_ub4);
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
	return mavlink_msg_shim_params_pack_chan(system_id, component_id, chan, msg, shim_params->d_ctrl, shim_params->smooth, shim_params->lookahead, shim_params->roll_lb, shim_params->abraking, shim_params->mid_throttle, shim_params->y_ub1, shim_params->y_lb1, shim_params->vy_ub1, shim_params->x_ub1, shim_params->x_lb1, shim_params->vx_ub1, shim_params->y_ub2, shim_params->y_lb2, shim_params->vy_ub2, shim_params->x_ub2, shim_params->x_lb2, shim_params->vx_ub2, shim_params->y_ub3, shim_params->y_lb3, shim_params->vy_ub3, shim_params->x_ub3, shim_params->x_lb3, shim_params->vx_ub3, shim_params->y_ub4, shim_params->y_lb4, shim_params->vy_ub4, shim_params->x_ub4, shim_params->x_lb4, shim_params->vx_ub4);
}

/**
 * @brief Send a shim_params message
 * @param chan MAVLink channel to send the message
 *
 * @param d_ctrl 1 to enable smoothing, 0 to disable
 * @param smooth 1 to enable smoothing, 0 to disable
 * @param lookahead lookahead factor for the smoothing shim
 * @param roll_lb the lower bound on roll angle allowed by the shim in (??DEGREES??)
 * @param abraking The braking acceleration in cm/s/s issued when the height-dimension shim is engaged. Should be at least the acceleration due to gravity.
 * @param mid_throttle The throttle required for hover.
 * @param y_ub1 the height upper bound of the shim in cm
 * @param y_lb1 the height lower bound of the shim in cm
 * @param vy_ub1 the vertical velocity upper bound of the shim in cm/s
 * @param x_ub1 the upper bound for the "x" dimension of the shim in cm
 * @param x_lb1 the lower bound for the "x" dimension of the shim in cm
 * @param vx_ub1 the "x" dimension velocity upper bound of the shim in cm/s
 * @param y_ub2 the height upper bound of the shim in cm
 * @param y_lb2 the height lower bound of the shim in cm
 * @param vy_ub2 the vertical velocity upper bound of the shim in cm/s
 * @param x_ub2 the upper bound for the "x" dimension of the shim in cm
 * @param x_lb2 the lower bound for the "x" dimension of the shim in cm
 * @param vx_ub2 the "x" dimension velocity upper bound of the shim in cm/s
 * @param y_ub3 the height upper bound of the shim in cm
 * @param y_lb3 the height lower bound of the shim in cm
 * @param vy_ub3 the vertical velocity upper bound of the shim in cm/s
 * @param x_ub3 the upper bound for the "x" dimension of the shim in cm
 * @param x_lb3 the lower bound for the "x" dimension of the shim in cm
 * @param vx_ub3 the "x" dimension velocity upper bound of the shim in cm/s
 * @param y_ub4 the height upper bound of the shim in cm
 * @param y_lb4 the height lower bound of the shim in cm
 * @param vy_ub4 the vertical velocity upper bound of the shim in cm/s
 * @param x_ub4 the upper bound for the "x" dimension of the shim in cm
 * @param x_lb4 the lower bound for the "x" dimension of the shim in cm
 * @param vx_ub4 the "x" dimension velocity upper bound of the shim in cm/s
 */
#ifdef MAVLINK_USE_CONVENIENCE_FUNCTIONS

static inline void mavlink_msg_shim_params_send(mavlink_channel_t chan, float d_ctrl, uint8_t smooth, uint8_t lookahead, float roll_lb, int16_t abraking, uint16_t mid_throttle, int16_t y_ub1, int16_t y_lb1, int16_t vy_ub1, int16_t x_ub1, int16_t x_lb1, int16_t vx_ub1, int16_t y_ub2, int16_t y_lb2, int16_t vy_ub2, int16_t x_ub2, int16_t x_lb2, int16_t vx_ub2, int16_t y_ub3, int16_t y_lb3, int16_t vy_ub3, int16_t x_ub3, int16_t x_lb3, int16_t vx_ub3, int16_t y_ub4, int16_t y_lb4, int16_t vy_ub4, int16_t x_ub4, int16_t x_lb4, int16_t vx_ub4)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
	char buf[MAVLINK_MSG_ID_SHIM_PARAMS_LEN];
	_mav_put_float(buf, 0, d_ctrl);
	_mav_put_float(buf, 4, roll_lb);
	_mav_put_int16_t(buf, 8, abraking);
	_mav_put_uint16_t(buf, 10, mid_throttle);
	_mav_put_int16_t(buf, 12, y_ub1);
	_mav_put_int16_t(buf, 14, y_lb1);
	_mav_put_int16_t(buf, 16, vy_ub1);
	_mav_put_int16_t(buf, 18, x_ub1);
	_mav_put_int16_t(buf, 20, x_lb1);
	_mav_put_int16_t(buf, 22, vx_ub1);
	_mav_put_int16_t(buf, 24, y_ub2);
	_mav_put_int16_t(buf, 26, y_lb2);
	_mav_put_int16_t(buf, 28, vy_ub2);
	_mav_put_int16_t(buf, 30, x_ub2);
	_mav_put_int16_t(buf, 32, x_lb2);
	_mav_put_int16_t(buf, 34, vx_ub2);
	_mav_put_int16_t(buf, 36, y_ub3);
	_mav_put_int16_t(buf, 38, y_lb3);
	_mav_put_int16_t(buf, 40, vy_ub3);
	_mav_put_int16_t(buf, 42, x_ub3);
	_mav_put_int16_t(buf, 44, x_lb3);
	_mav_put_int16_t(buf, 46, vx_ub3);
	_mav_put_int16_t(buf, 48, y_ub4);
	_mav_put_int16_t(buf, 50, y_lb4);
	_mav_put_int16_t(buf, 52, vy_ub4);
	_mav_put_int16_t(buf, 54, x_ub4);
	_mav_put_int16_t(buf, 56, x_lb4);
	_mav_put_int16_t(buf, 58, vx_ub4);
	_mav_put_uint8_t(buf, 60, smooth);
	_mav_put_uint8_t(buf, 61, lookahead);

#if MAVLINK_CRC_EXTRA
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_SHIM_PARAMS, buf, MAVLINK_MSG_ID_SHIM_PARAMS_LEN, MAVLINK_MSG_ID_SHIM_PARAMS_CRC);
#else
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_SHIM_PARAMS, buf, MAVLINK_MSG_ID_SHIM_PARAMS_LEN);
#endif
#else
	mavlink_shim_params_t packet;
	packet.d_ctrl = d_ctrl;
	packet.roll_lb = roll_lb;
	packet.abraking = abraking;
	packet.mid_throttle = mid_throttle;
	packet.y_ub1 = y_ub1;
	packet.y_lb1 = y_lb1;
	packet.vy_ub1 = vy_ub1;
	packet.x_ub1 = x_ub1;
	packet.x_lb1 = x_lb1;
	packet.vx_ub1 = vx_ub1;
	packet.y_ub2 = y_ub2;
	packet.y_lb2 = y_lb2;
	packet.vy_ub2 = vy_ub2;
	packet.x_ub2 = x_ub2;
	packet.x_lb2 = x_lb2;
	packet.vx_ub2 = vx_ub2;
	packet.y_ub3 = y_ub3;
	packet.y_lb3 = y_lb3;
	packet.vy_ub3 = vy_ub3;
	packet.x_ub3 = x_ub3;
	packet.x_lb3 = x_lb3;
	packet.vx_ub3 = vx_ub3;
	packet.y_ub4 = y_ub4;
	packet.y_lb4 = y_lb4;
	packet.vy_ub4 = vy_ub4;
	packet.x_ub4 = x_ub4;
	packet.x_lb4 = x_lb4;
	packet.vx_ub4 = vx_ub4;
	packet.smooth = smooth;
	packet.lookahead = lookahead;

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
static inline void mavlink_msg_shim_params_send_buf(mavlink_message_t *msgbuf, mavlink_channel_t chan,  float d_ctrl, uint8_t smooth, uint8_t lookahead, float roll_lb, int16_t abraking, uint16_t mid_throttle, int16_t y_ub1, int16_t y_lb1, int16_t vy_ub1, int16_t x_ub1, int16_t x_lb1, int16_t vx_ub1, int16_t y_ub2, int16_t y_lb2, int16_t vy_ub2, int16_t x_ub2, int16_t x_lb2, int16_t vx_ub2, int16_t y_ub3, int16_t y_lb3, int16_t vy_ub3, int16_t x_ub3, int16_t x_lb3, int16_t vx_ub3, int16_t y_ub4, int16_t y_lb4, int16_t vy_ub4, int16_t x_ub4, int16_t x_lb4, int16_t vx_ub4)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
	char *buf = (char *)msgbuf;
	_mav_put_float(buf, 0, d_ctrl);
	_mav_put_float(buf, 4, roll_lb);
	_mav_put_int16_t(buf, 8, abraking);
	_mav_put_uint16_t(buf, 10, mid_throttle);
	_mav_put_int16_t(buf, 12, y_ub1);
	_mav_put_int16_t(buf, 14, y_lb1);
	_mav_put_int16_t(buf, 16, vy_ub1);
	_mav_put_int16_t(buf, 18, x_ub1);
	_mav_put_int16_t(buf, 20, x_lb1);
	_mav_put_int16_t(buf, 22, vx_ub1);
	_mav_put_int16_t(buf, 24, y_ub2);
	_mav_put_int16_t(buf, 26, y_lb2);
	_mav_put_int16_t(buf, 28, vy_ub2);
	_mav_put_int16_t(buf, 30, x_ub2);
	_mav_put_int16_t(buf, 32, x_lb2);
	_mav_put_int16_t(buf, 34, vx_ub2);
	_mav_put_int16_t(buf, 36, y_ub3);
	_mav_put_int16_t(buf, 38, y_lb3);
	_mav_put_int16_t(buf, 40, vy_ub3);
	_mav_put_int16_t(buf, 42, x_ub3);
	_mav_put_int16_t(buf, 44, x_lb3);
	_mav_put_int16_t(buf, 46, vx_ub3);
	_mav_put_int16_t(buf, 48, y_ub4);
	_mav_put_int16_t(buf, 50, y_lb4);
	_mav_put_int16_t(buf, 52, vy_ub4);
	_mav_put_int16_t(buf, 54, x_ub4);
	_mav_put_int16_t(buf, 56, x_lb4);
	_mav_put_int16_t(buf, 58, vx_ub4);
	_mav_put_uint8_t(buf, 60, smooth);
	_mav_put_uint8_t(buf, 61, lookahead);

#if MAVLINK_CRC_EXTRA
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_SHIM_PARAMS, buf, MAVLINK_MSG_ID_SHIM_PARAMS_LEN, MAVLINK_MSG_ID_SHIM_PARAMS_CRC);
#else
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_SHIM_PARAMS, buf, MAVLINK_MSG_ID_SHIM_PARAMS_LEN);
#endif
#else
	mavlink_shim_params_t *packet = (mavlink_shim_params_t *)msgbuf;
	packet->d_ctrl = d_ctrl;
	packet->roll_lb = roll_lb;
	packet->abraking = abraking;
	packet->mid_throttle = mid_throttle;
	packet->y_ub1 = y_ub1;
	packet->y_lb1 = y_lb1;
	packet->vy_ub1 = vy_ub1;
	packet->x_ub1 = x_ub1;
	packet->x_lb1 = x_lb1;
	packet->vx_ub1 = vx_ub1;
	packet->y_ub2 = y_ub2;
	packet->y_lb2 = y_lb2;
	packet->vy_ub2 = vy_ub2;
	packet->x_ub2 = x_ub2;
	packet->x_lb2 = x_lb2;
	packet->vx_ub2 = vx_ub2;
	packet->y_ub3 = y_ub3;
	packet->y_lb3 = y_lb3;
	packet->vy_ub3 = vy_ub3;
	packet->x_ub3 = x_ub3;
	packet->x_lb3 = x_lb3;
	packet->vx_ub3 = vx_ub3;
	packet->y_ub4 = y_ub4;
	packet->y_lb4 = y_lb4;
	packet->vy_ub4 = vy_ub4;
	packet->x_ub4 = x_ub4;
	packet->x_lb4 = x_lb4;
	packet->vx_ub4 = vx_ub4;
	packet->smooth = smooth;
	packet->lookahead = lookahead;

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
 * @brief Get field d_ctrl from shim_params message
 *
 * @return 1 to enable smoothing, 0 to disable
 */
static inline float mavlink_msg_shim_params_get_d_ctrl(const mavlink_message_t* msg)
{
	return _MAV_RETURN_float(msg,  0);
}

/**
 * @brief Get field smooth from shim_params message
 *
 * @return 1 to enable smoothing, 0 to disable
 */
static inline uint8_t mavlink_msg_shim_params_get_smooth(const mavlink_message_t* msg)
{
	return _MAV_RETURN_uint8_t(msg,  60);
}

/**
 * @brief Get field lookahead from shim_params message
 *
 * @return lookahead factor for the smoothing shim
 */
static inline uint8_t mavlink_msg_shim_params_get_lookahead(const mavlink_message_t* msg)
{
	return _MAV_RETURN_uint8_t(msg,  61);
}

/**
 * @brief Get field roll_lb from shim_params message
 *
 * @return the lower bound on roll angle allowed by the shim in (??DEGREES??)
 */
static inline float mavlink_msg_shim_params_get_roll_lb(const mavlink_message_t* msg)
{
	return _MAV_RETURN_float(msg,  4);
}

/**
 * @brief Get field abraking from shim_params message
 *
 * @return The braking acceleration in cm/s/s issued when the height-dimension shim is engaged. Should be at least the acceleration due to gravity.
 */
static inline int16_t mavlink_msg_shim_params_get_abraking(const mavlink_message_t* msg)
{
	return _MAV_RETURN_int16_t(msg,  8);
}

/**
 * @brief Get field mid_throttle from shim_params message
 *
 * @return The throttle required for hover.
 */
static inline uint16_t mavlink_msg_shim_params_get_mid_throttle(const mavlink_message_t* msg)
{
	return _MAV_RETURN_uint16_t(msg,  10);
}

/**
 * @brief Get field y_ub1 from shim_params message
 *
 * @return the height upper bound of the shim in cm
 */
static inline int16_t mavlink_msg_shim_params_get_y_ub1(const mavlink_message_t* msg)
{
	return _MAV_RETURN_int16_t(msg,  12);
}

/**
 * @brief Get field y_lb1 from shim_params message
 *
 * @return the height lower bound of the shim in cm
 */
static inline int16_t mavlink_msg_shim_params_get_y_lb1(const mavlink_message_t* msg)
{
	return _MAV_RETURN_int16_t(msg,  14);
}

/**
 * @brief Get field vy_ub1 from shim_params message
 *
 * @return the vertical velocity upper bound of the shim in cm/s
 */
static inline int16_t mavlink_msg_shim_params_get_vy_ub1(const mavlink_message_t* msg)
{
	return _MAV_RETURN_int16_t(msg,  16);
}

/**
 * @brief Get field x_ub1 from shim_params message
 *
 * @return the upper bound for the "x" dimension of the shim in cm
 */
static inline int16_t mavlink_msg_shim_params_get_x_ub1(const mavlink_message_t* msg)
{
	return _MAV_RETURN_int16_t(msg,  18);
}

/**
 * @brief Get field x_lb1 from shim_params message
 *
 * @return the lower bound for the "x" dimension of the shim in cm
 */
static inline int16_t mavlink_msg_shim_params_get_x_lb1(const mavlink_message_t* msg)
{
	return _MAV_RETURN_int16_t(msg,  20);
}

/**
 * @brief Get field vx_ub1 from shim_params message
 *
 * @return the "x" dimension velocity upper bound of the shim in cm/s
 */
static inline int16_t mavlink_msg_shim_params_get_vx_ub1(const mavlink_message_t* msg)
{
	return _MAV_RETURN_int16_t(msg,  22);
}

/**
 * @brief Get field y_ub2 from shim_params message
 *
 * @return the height upper bound of the shim in cm
 */
static inline int16_t mavlink_msg_shim_params_get_y_ub2(const mavlink_message_t* msg)
{
	return _MAV_RETURN_int16_t(msg,  24);
}

/**
 * @brief Get field y_lb2 from shim_params message
 *
 * @return the height lower bound of the shim in cm
 */
static inline int16_t mavlink_msg_shim_params_get_y_lb2(const mavlink_message_t* msg)
{
	return _MAV_RETURN_int16_t(msg,  26);
}

/**
 * @brief Get field vy_ub2 from shim_params message
 *
 * @return the vertical velocity upper bound of the shim in cm/s
 */
static inline int16_t mavlink_msg_shim_params_get_vy_ub2(const mavlink_message_t* msg)
{
	return _MAV_RETURN_int16_t(msg,  28);
}

/**
 * @brief Get field x_ub2 from shim_params message
 *
 * @return the upper bound for the "x" dimension of the shim in cm
 */
static inline int16_t mavlink_msg_shim_params_get_x_ub2(const mavlink_message_t* msg)
{
	return _MAV_RETURN_int16_t(msg,  30);
}

/**
 * @brief Get field x_lb2 from shim_params message
 *
 * @return the lower bound for the "x" dimension of the shim in cm
 */
static inline int16_t mavlink_msg_shim_params_get_x_lb2(const mavlink_message_t* msg)
{
	return _MAV_RETURN_int16_t(msg,  32);
}

/**
 * @brief Get field vx_ub2 from shim_params message
 *
 * @return the "x" dimension velocity upper bound of the shim in cm/s
 */
static inline int16_t mavlink_msg_shim_params_get_vx_ub2(const mavlink_message_t* msg)
{
	return _MAV_RETURN_int16_t(msg,  34);
}

/**
 * @brief Get field y_ub3 from shim_params message
 *
 * @return the height upper bound of the shim in cm
 */
static inline int16_t mavlink_msg_shim_params_get_y_ub3(const mavlink_message_t* msg)
{
	return _MAV_RETURN_int16_t(msg,  36);
}

/**
 * @brief Get field y_lb3 from shim_params message
 *
 * @return the height lower bound of the shim in cm
 */
static inline int16_t mavlink_msg_shim_params_get_y_lb3(const mavlink_message_t* msg)
{
	return _MAV_RETURN_int16_t(msg,  38);
}

/**
 * @brief Get field vy_ub3 from shim_params message
 *
 * @return the vertical velocity upper bound of the shim in cm/s
 */
static inline int16_t mavlink_msg_shim_params_get_vy_ub3(const mavlink_message_t* msg)
{
	return _MAV_RETURN_int16_t(msg,  40);
}

/**
 * @brief Get field x_ub3 from shim_params message
 *
 * @return the upper bound for the "x" dimension of the shim in cm
 */
static inline int16_t mavlink_msg_shim_params_get_x_ub3(const mavlink_message_t* msg)
{
	return _MAV_RETURN_int16_t(msg,  42);
}

/**
 * @brief Get field x_lb3 from shim_params message
 *
 * @return the lower bound for the "x" dimension of the shim in cm
 */
static inline int16_t mavlink_msg_shim_params_get_x_lb3(const mavlink_message_t* msg)
{
	return _MAV_RETURN_int16_t(msg,  44);
}

/**
 * @brief Get field vx_ub3 from shim_params message
 *
 * @return the "x" dimension velocity upper bound of the shim in cm/s
 */
static inline int16_t mavlink_msg_shim_params_get_vx_ub3(const mavlink_message_t* msg)
{
	return _MAV_RETURN_int16_t(msg,  46);
}

/**
 * @brief Get field y_ub4 from shim_params message
 *
 * @return the height upper bound of the shim in cm
 */
static inline int16_t mavlink_msg_shim_params_get_y_ub4(const mavlink_message_t* msg)
{
	return _MAV_RETURN_int16_t(msg,  48);
}

/**
 * @brief Get field y_lb4 from shim_params message
 *
 * @return the height lower bound of the shim in cm
 */
static inline int16_t mavlink_msg_shim_params_get_y_lb4(const mavlink_message_t* msg)
{
	return _MAV_RETURN_int16_t(msg,  50);
}

/**
 * @brief Get field vy_ub4 from shim_params message
 *
 * @return the vertical velocity upper bound of the shim in cm/s
 */
static inline int16_t mavlink_msg_shim_params_get_vy_ub4(const mavlink_message_t* msg)
{
	return _MAV_RETURN_int16_t(msg,  52);
}

/**
 * @brief Get field x_ub4 from shim_params message
 *
 * @return the upper bound for the "x" dimension of the shim in cm
 */
static inline int16_t mavlink_msg_shim_params_get_x_ub4(const mavlink_message_t* msg)
{
	return _MAV_RETURN_int16_t(msg,  54);
}

/**
 * @brief Get field x_lb4 from shim_params message
 *
 * @return the lower bound for the "x" dimension of the shim in cm
 */
static inline int16_t mavlink_msg_shim_params_get_x_lb4(const mavlink_message_t* msg)
{
	return _MAV_RETURN_int16_t(msg,  56);
}

/**
 * @brief Get field vx_ub4 from shim_params message
 *
 * @return the "x" dimension velocity upper bound of the shim in cm/s
 */
static inline int16_t mavlink_msg_shim_params_get_vx_ub4(const mavlink_message_t* msg)
{
	return _MAV_RETURN_int16_t(msg,  58);
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
	shim_params->d_ctrl = mavlink_msg_shim_params_get_d_ctrl(msg);
	shim_params->roll_lb = mavlink_msg_shim_params_get_roll_lb(msg);
	shim_params->abraking = mavlink_msg_shim_params_get_abraking(msg);
	shim_params->mid_throttle = mavlink_msg_shim_params_get_mid_throttle(msg);
	shim_params->y_ub1 = mavlink_msg_shim_params_get_y_ub1(msg);
	shim_params->y_lb1 = mavlink_msg_shim_params_get_y_lb1(msg);
	shim_params->vy_ub1 = mavlink_msg_shim_params_get_vy_ub1(msg);
	shim_params->x_ub1 = mavlink_msg_shim_params_get_x_ub1(msg);
	shim_params->x_lb1 = mavlink_msg_shim_params_get_x_lb1(msg);
	shim_params->vx_ub1 = mavlink_msg_shim_params_get_vx_ub1(msg);
	shim_params->y_ub2 = mavlink_msg_shim_params_get_y_ub2(msg);
	shim_params->y_lb2 = mavlink_msg_shim_params_get_y_lb2(msg);
	shim_params->vy_ub2 = mavlink_msg_shim_params_get_vy_ub2(msg);
	shim_params->x_ub2 = mavlink_msg_shim_params_get_x_ub2(msg);
	shim_params->x_lb2 = mavlink_msg_shim_params_get_x_lb2(msg);
	shim_params->vx_ub2 = mavlink_msg_shim_params_get_vx_ub2(msg);
	shim_params->y_ub3 = mavlink_msg_shim_params_get_y_ub3(msg);
	shim_params->y_lb3 = mavlink_msg_shim_params_get_y_lb3(msg);
	shim_params->vy_ub3 = mavlink_msg_shim_params_get_vy_ub3(msg);
	shim_params->x_ub3 = mavlink_msg_shim_params_get_x_ub3(msg);
	shim_params->x_lb3 = mavlink_msg_shim_params_get_x_lb3(msg);
	shim_params->vx_ub3 = mavlink_msg_shim_params_get_vx_ub3(msg);
	shim_params->y_ub4 = mavlink_msg_shim_params_get_y_ub4(msg);
	shim_params->y_lb4 = mavlink_msg_shim_params_get_y_lb4(msg);
	shim_params->vy_ub4 = mavlink_msg_shim_params_get_vy_ub4(msg);
	shim_params->x_ub4 = mavlink_msg_shim_params_get_x_ub4(msg);
	shim_params->x_lb4 = mavlink_msg_shim_params_get_x_lb4(msg);
	shim_params->vx_ub4 = mavlink_msg_shim_params_get_vx_ub4(msg);
	shim_params->smooth = mavlink_msg_shim_params_get_smooth(msg);
	shim_params->lookahead = mavlink_msg_shim_params_get_lookahead(msg);
#else
	memcpy(shim_params, _MAV_PAYLOAD(msg), MAVLINK_MSG_ID_SHIM_PARAMS_LEN);
#endif
}
