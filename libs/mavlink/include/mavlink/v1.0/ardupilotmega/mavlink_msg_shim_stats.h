// MESSAGE SHIM_STATS PACKING

#define MAVLINK_MSG_ID_SHIM_STATS 232

typedef struct __mavlink_shim_stats_t
{
 float x; ///< The value of x, relative to starting position.
 float y; ///< The value of y, relative to starting position.
 float z; ///< The height, relative to starting position.
 float vx; ///< The velocity in the x direction.
 float vy; ///< The velocity in the y direction.
 float vz; ///< The velocity in the vertical direction.
 float A_proposed; ///< The proposed acceleration, normal to the quadcopter.
 float Roll_proposed; ///< The proposed roll angle
 float Pitch_proposed; ///< The proposed pitch angle
 float a; ///< The issued acceleration, normal to the quadcopter.
 float roll; ///< The issued roll angle
 float pitch; ///< The issued pitch angle
 float AX_proposed; ///< The proposed AX
 float AY_proposed; ///< The proposed AY
 float AZ_proposed; ///< The proposed AZ
 float ax; ///< The actual ax
 float ay; ///< The actual ay
 float az; ///< The actual az
 uint16_t throttle; ///< The proposed throttle received by the shim
 uint8_t can_run1; ///< 1 if shim 1 can run, 0 otherwise
 uint8_t can_run2; ///< 1 if shim 2 can run, 0 otherwise
 uint8_t can_run3; ///< 1 if shim 3 can run, 0 otherwise
 uint8_t can_run4; ///< 1 if shim 4 can run, 0 otherwise
 uint8_t angle_boost; ///< 1 if angle boost was requested, 0 otherwise
 uint8_t last_shim; ///< The last shim to run
 uint8_t safe_x; ///< 1 if safe in x dimension, 0 otherwise
 uint8_t safe_y; ///< 1 if safe in y dimension, 0 otherwise
 uint8_t safe_z; ///< 1 if safe in z dimension, 0 otherwise
} mavlink_shim_stats_t;

#define MAVLINK_MSG_ID_SHIM_STATS_LEN 83
#define MAVLINK_MSG_ID_232_LEN 83

#define MAVLINK_MSG_ID_SHIM_STATS_CRC 154
#define MAVLINK_MSG_ID_232_CRC 154



#define MAVLINK_MESSAGE_INFO_SHIM_STATS { \
	"SHIM_STATS", \
	28, \
	{  { "x", NULL, MAVLINK_TYPE_FLOAT, 0, 0, offsetof(mavlink_shim_stats_t, x) }, \
         { "y", NULL, MAVLINK_TYPE_FLOAT, 0, 4, offsetof(mavlink_shim_stats_t, y) }, \
         { "z", NULL, MAVLINK_TYPE_FLOAT, 0, 8, offsetof(mavlink_shim_stats_t, z) }, \
         { "vx", NULL, MAVLINK_TYPE_FLOAT, 0, 12, offsetof(mavlink_shim_stats_t, vx) }, \
         { "vy", NULL, MAVLINK_TYPE_FLOAT, 0, 16, offsetof(mavlink_shim_stats_t, vy) }, \
         { "vz", NULL, MAVLINK_TYPE_FLOAT, 0, 20, offsetof(mavlink_shim_stats_t, vz) }, \
         { "A_proposed", NULL, MAVLINK_TYPE_FLOAT, 0, 24, offsetof(mavlink_shim_stats_t, A_proposed) }, \
         { "Roll_proposed", NULL, MAVLINK_TYPE_FLOAT, 0, 28, offsetof(mavlink_shim_stats_t, Roll_proposed) }, \
         { "Pitch_proposed", NULL, MAVLINK_TYPE_FLOAT, 0, 32, offsetof(mavlink_shim_stats_t, Pitch_proposed) }, \
         { "a", NULL, MAVLINK_TYPE_FLOAT, 0, 36, offsetof(mavlink_shim_stats_t, a) }, \
         { "roll", NULL, MAVLINK_TYPE_FLOAT, 0, 40, offsetof(mavlink_shim_stats_t, roll) }, \
         { "pitch", NULL, MAVLINK_TYPE_FLOAT, 0, 44, offsetof(mavlink_shim_stats_t, pitch) }, \
         { "AX_proposed", NULL, MAVLINK_TYPE_FLOAT, 0, 48, offsetof(mavlink_shim_stats_t, AX_proposed) }, \
         { "AY_proposed", NULL, MAVLINK_TYPE_FLOAT, 0, 52, offsetof(mavlink_shim_stats_t, AY_proposed) }, \
         { "AZ_proposed", NULL, MAVLINK_TYPE_FLOAT, 0, 56, offsetof(mavlink_shim_stats_t, AZ_proposed) }, \
         { "ax", NULL, MAVLINK_TYPE_FLOAT, 0, 60, offsetof(mavlink_shim_stats_t, ax) }, \
         { "ay", NULL, MAVLINK_TYPE_FLOAT, 0, 64, offsetof(mavlink_shim_stats_t, ay) }, \
         { "az", NULL, MAVLINK_TYPE_FLOAT, 0, 68, offsetof(mavlink_shim_stats_t, az) }, \
         { "throttle", NULL, MAVLINK_TYPE_UINT16_T, 0, 72, offsetof(mavlink_shim_stats_t, throttle) }, \
         { "can_run1", NULL, MAVLINK_TYPE_UINT8_T, 0, 74, offsetof(mavlink_shim_stats_t, can_run1) }, \
         { "can_run2", NULL, MAVLINK_TYPE_UINT8_T, 0, 75, offsetof(mavlink_shim_stats_t, can_run2) }, \
         { "can_run3", NULL, MAVLINK_TYPE_UINT8_T, 0, 76, offsetof(mavlink_shim_stats_t, can_run3) }, \
         { "can_run4", NULL, MAVLINK_TYPE_UINT8_T, 0, 77, offsetof(mavlink_shim_stats_t, can_run4) }, \
         { "angle_boost", NULL, MAVLINK_TYPE_UINT8_T, 0, 78, offsetof(mavlink_shim_stats_t, angle_boost) }, \
         { "last_shim", NULL, MAVLINK_TYPE_UINT8_T, 0, 79, offsetof(mavlink_shim_stats_t, last_shim) }, \
         { "safe_x", NULL, MAVLINK_TYPE_UINT8_T, 0, 80, offsetof(mavlink_shim_stats_t, safe_x) }, \
         { "safe_y", NULL, MAVLINK_TYPE_UINT8_T, 0, 81, offsetof(mavlink_shim_stats_t, safe_y) }, \
         { "safe_z", NULL, MAVLINK_TYPE_UINT8_T, 0, 82, offsetof(mavlink_shim_stats_t, safe_z) }, \
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
 * @param y The value of y, relative to starting position.
 * @param z The height, relative to starting position.
 * @param vx The velocity in the x direction.
 * @param vy The velocity in the y direction.
 * @param vz The velocity in the vertical direction.
 * @param throttle The proposed throttle received by the shim
 * @param angle_boost 1 if angle boost was requested, 0 otherwise
 * @param A_proposed The proposed acceleration, normal to the quadcopter.
 * @param Roll_proposed The proposed roll angle
 * @param Pitch_proposed The proposed pitch angle
 * @param a The issued acceleration, normal to the quadcopter.
 * @param roll The issued roll angle
 * @param pitch The issued pitch angle
 * @param last_shim The last shim to run
 * @param AX_proposed The proposed AX
 * @param AY_proposed The proposed AY
 * @param AZ_proposed The proposed AZ
 * @param ax The actual ax
 * @param ay The actual ay
 * @param az The actual az
 * @param safe_x 1 if safe in x dimension, 0 otherwise
 * @param safe_y 1 if safe in y dimension, 0 otherwise
 * @param safe_z 1 if safe in z dimension, 0 otherwise
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t mavlink_msg_shim_stats_pack(uint8_t system_id, uint8_t component_id, mavlink_message_t* msg,
						       uint8_t can_run1, uint8_t can_run2, uint8_t can_run3, uint8_t can_run4, float x, float y, float z, float vx, float vy, float vz, uint16_t throttle, uint8_t angle_boost, float A_proposed, float Roll_proposed, float Pitch_proposed, float a, float roll, float pitch, uint8_t last_shim, float AX_proposed, float AY_proposed, float AZ_proposed, float ax, float ay, float az, uint8_t safe_x, uint8_t safe_y, uint8_t safe_z)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
	char buf[MAVLINK_MSG_ID_SHIM_STATS_LEN];
	_mav_put_float(buf, 0, x);
	_mav_put_float(buf, 4, y);
	_mav_put_float(buf, 8, z);
	_mav_put_float(buf, 12, vx);
	_mav_put_float(buf, 16, vy);
	_mav_put_float(buf, 20, vz);
	_mav_put_float(buf, 24, A_proposed);
	_mav_put_float(buf, 28, Roll_proposed);
	_mav_put_float(buf, 32, Pitch_proposed);
	_mav_put_float(buf, 36, a);
	_mav_put_float(buf, 40, roll);
	_mav_put_float(buf, 44, pitch);
	_mav_put_float(buf, 48, AX_proposed);
	_mav_put_float(buf, 52, AY_proposed);
	_mav_put_float(buf, 56, AZ_proposed);
	_mav_put_float(buf, 60, ax);
	_mav_put_float(buf, 64, ay);
	_mav_put_float(buf, 68, az);
	_mav_put_uint16_t(buf, 72, throttle);
	_mav_put_uint8_t(buf, 74, can_run1);
	_mav_put_uint8_t(buf, 75, can_run2);
	_mav_put_uint8_t(buf, 76, can_run3);
	_mav_put_uint8_t(buf, 77, can_run4);
	_mav_put_uint8_t(buf, 78, angle_boost);
	_mav_put_uint8_t(buf, 79, last_shim);
	_mav_put_uint8_t(buf, 80, safe_x);
	_mav_put_uint8_t(buf, 81, safe_y);
	_mav_put_uint8_t(buf, 82, safe_z);

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), buf, MAVLINK_MSG_ID_SHIM_STATS_LEN);
#else
	mavlink_shim_stats_t packet;
	packet.x = x;
	packet.y = y;
	packet.z = z;
	packet.vx = vx;
	packet.vy = vy;
	packet.vz = vz;
	packet.A_proposed = A_proposed;
	packet.Roll_proposed = Roll_proposed;
	packet.Pitch_proposed = Pitch_proposed;
	packet.a = a;
	packet.roll = roll;
	packet.pitch = pitch;
	packet.AX_proposed = AX_proposed;
	packet.AY_proposed = AY_proposed;
	packet.AZ_proposed = AZ_proposed;
	packet.ax = ax;
	packet.ay = ay;
	packet.az = az;
	packet.throttle = throttle;
	packet.can_run1 = can_run1;
	packet.can_run2 = can_run2;
	packet.can_run3 = can_run3;
	packet.can_run4 = can_run4;
	packet.angle_boost = angle_boost;
	packet.last_shim = last_shim;
	packet.safe_x = safe_x;
	packet.safe_y = safe_y;
	packet.safe_z = safe_z;

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
 * @param y The value of y, relative to starting position.
 * @param z The height, relative to starting position.
 * @param vx The velocity in the x direction.
 * @param vy The velocity in the y direction.
 * @param vz The velocity in the vertical direction.
 * @param throttle The proposed throttle received by the shim
 * @param angle_boost 1 if angle boost was requested, 0 otherwise
 * @param A_proposed The proposed acceleration, normal to the quadcopter.
 * @param Roll_proposed The proposed roll angle
 * @param Pitch_proposed The proposed pitch angle
 * @param a The issued acceleration, normal to the quadcopter.
 * @param roll The issued roll angle
 * @param pitch The issued pitch angle
 * @param last_shim The last shim to run
 * @param AX_proposed The proposed AX
 * @param AY_proposed The proposed AY
 * @param AZ_proposed The proposed AZ
 * @param ax The actual ax
 * @param ay The actual ay
 * @param az The actual az
 * @param safe_x 1 if safe in x dimension, 0 otherwise
 * @param safe_y 1 if safe in y dimension, 0 otherwise
 * @param safe_z 1 if safe in z dimension, 0 otherwise
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t mavlink_msg_shim_stats_pack_chan(uint8_t system_id, uint8_t component_id, uint8_t chan,
							   mavlink_message_t* msg,
						           uint8_t can_run1,uint8_t can_run2,uint8_t can_run3,uint8_t can_run4,float x,float y,float z,float vx,float vy,float vz,uint16_t throttle,uint8_t angle_boost,float A_proposed,float Roll_proposed,float Pitch_proposed,float a,float roll,float pitch,uint8_t last_shim,float AX_proposed,float AY_proposed,float AZ_proposed,float ax,float ay,float az,uint8_t safe_x,uint8_t safe_y,uint8_t safe_z)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
	char buf[MAVLINK_MSG_ID_SHIM_STATS_LEN];
	_mav_put_float(buf, 0, x);
	_mav_put_float(buf, 4, y);
	_mav_put_float(buf, 8, z);
	_mav_put_float(buf, 12, vx);
	_mav_put_float(buf, 16, vy);
	_mav_put_float(buf, 20, vz);
	_mav_put_float(buf, 24, A_proposed);
	_mav_put_float(buf, 28, Roll_proposed);
	_mav_put_float(buf, 32, Pitch_proposed);
	_mav_put_float(buf, 36, a);
	_mav_put_float(buf, 40, roll);
	_mav_put_float(buf, 44, pitch);
	_mav_put_float(buf, 48, AX_proposed);
	_mav_put_float(buf, 52, AY_proposed);
	_mav_put_float(buf, 56, AZ_proposed);
	_mav_put_float(buf, 60, ax);
	_mav_put_float(buf, 64, ay);
	_mav_put_float(buf, 68, az);
	_mav_put_uint16_t(buf, 72, throttle);
	_mav_put_uint8_t(buf, 74, can_run1);
	_mav_put_uint8_t(buf, 75, can_run2);
	_mav_put_uint8_t(buf, 76, can_run3);
	_mav_put_uint8_t(buf, 77, can_run4);
	_mav_put_uint8_t(buf, 78, angle_boost);
	_mav_put_uint8_t(buf, 79, last_shim);
	_mav_put_uint8_t(buf, 80, safe_x);
	_mav_put_uint8_t(buf, 81, safe_y);
	_mav_put_uint8_t(buf, 82, safe_z);

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), buf, MAVLINK_MSG_ID_SHIM_STATS_LEN);
#else
	mavlink_shim_stats_t packet;
	packet.x = x;
	packet.y = y;
	packet.z = z;
	packet.vx = vx;
	packet.vy = vy;
	packet.vz = vz;
	packet.A_proposed = A_proposed;
	packet.Roll_proposed = Roll_proposed;
	packet.Pitch_proposed = Pitch_proposed;
	packet.a = a;
	packet.roll = roll;
	packet.pitch = pitch;
	packet.AX_proposed = AX_proposed;
	packet.AY_proposed = AY_proposed;
	packet.AZ_proposed = AZ_proposed;
	packet.ax = ax;
	packet.ay = ay;
	packet.az = az;
	packet.throttle = throttle;
	packet.can_run1 = can_run1;
	packet.can_run2 = can_run2;
	packet.can_run3 = can_run3;
	packet.can_run4 = can_run4;
	packet.angle_boost = angle_boost;
	packet.last_shim = last_shim;
	packet.safe_x = safe_x;
	packet.safe_y = safe_y;
	packet.safe_z = safe_z;

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
	return mavlink_msg_shim_stats_pack(system_id, component_id, msg, shim_stats->can_run1, shim_stats->can_run2, shim_stats->can_run3, shim_stats->can_run4, shim_stats->x, shim_stats->y, shim_stats->z, shim_stats->vx, shim_stats->vy, shim_stats->vz, shim_stats->throttle, shim_stats->angle_boost, shim_stats->A_proposed, shim_stats->Roll_proposed, shim_stats->Pitch_proposed, shim_stats->a, shim_stats->roll, shim_stats->pitch, shim_stats->last_shim, shim_stats->AX_proposed, shim_stats->AY_proposed, shim_stats->AZ_proposed, shim_stats->ax, shim_stats->ay, shim_stats->az, shim_stats->safe_x, shim_stats->safe_y, shim_stats->safe_z);
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
	return mavlink_msg_shim_stats_pack_chan(system_id, component_id, chan, msg, shim_stats->can_run1, shim_stats->can_run2, shim_stats->can_run3, shim_stats->can_run4, shim_stats->x, shim_stats->y, shim_stats->z, shim_stats->vx, shim_stats->vy, shim_stats->vz, shim_stats->throttle, shim_stats->angle_boost, shim_stats->A_proposed, shim_stats->Roll_proposed, shim_stats->Pitch_proposed, shim_stats->a, shim_stats->roll, shim_stats->pitch, shim_stats->last_shim, shim_stats->AX_proposed, shim_stats->AY_proposed, shim_stats->AZ_proposed, shim_stats->ax, shim_stats->ay, shim_stats->az, shim_stats->safe_x, shim_stats->safe_y, shim_stats->safe_z);
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
 * @param y The value of y, relative to starting position.
 * @param z The height, relative to starting position.
 * @param vx The velocity in the x direction.
 * @param vy The velocity in the y direction.
 * @param vz The velocity in the vertical direction.
 * @param throttle The proposed throttle received by the shim
 * @param angle_boost 1 if angle boost was requested, 0 otherwise
 * @param A_proposed The proposed acceleration, normal to the quadcopter.
 * @param Roll_proposed The proposed roll angle
 * @param Pitch_proposed The proposed pitch angle
 * @param a The issued acceleration, normal to the quadcopter.
 * @param roll The issued roll angle
 * @param pitch The issued pitch angle
 * @param last_shim The last shim to run
 * @param AX_proposed The proposed AX
 * @param AY_proposed The proposed AY
 * @param AZ_proposed The proposed AZ
 * @param ax The actual ax
 * @param ay The actual ay
 * @param az The actual az
 * @param safe_x 1 if safe in x dimension, 0 otherwise
 * @param safe_y 1 if safe in y dimension, 0 otherwise
 * @param safe_z 1 if safe in z dimension, 0 otherwise
 */
#ifdef MAVLINK_USE_CONVENIENCE_FUNCTIONS

static inline void mavlink_msg_shim_stats_send(mavlink_channel_t chan, uint8_t can_run1, uint8_t can_run2, uint8_t can_run3, uint8_t can_run4, float x, float y, float z, float vx, float vy, float vz, uint16_t throttle, uint8_t angle_boost, float A_proposed, float Roll_proposed, float Pitch_proposed, float a, float roll, float pitch, uint8_t last_shim, float AX_proposed, float AY_proposed, float AZ_proposed, float ax, float ay, float az, uint8_t safe_x, uint8_t safe_y, uint8_t safe_z)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
	char buf[MAVLINK_MSG_ID_SHIM_STATS_LEN];
	_mav_put_float(buf, 0, x);
	_mav_put_float(buf, 4, y);
	_mav_put_float(buf, 8, z);
	_mav_put_float(buf, 12, vx);
	_mav_put_float(buf, 16, vy);
	_mav_put_float(buf, 20, vz);
	_mav_put_float(buf, 24, A_proposed);
	_mav_put_float(buf, 28, Roll_proposed);
	_mav_put_float(buf, 32, Pitch_proposed);
	_mav_put_float(buf, 36, a);
	_mav_put_float(buf, 40, roll);
	_mav_put_float(buf, 44, pitch);
	_mav_put_float(buf, 48, AX_proposed);
	_mav_put_float(buf, 52, AY_proposed);
	_mav_put_float(buf, 56, AZ_proposed);
	_mav_put_float(buf, 60, ax);
	_mav_put_float(buf, 64, ay);
	_mav_put_float(buf, 68, az);
	_mav_put_uint16_t(buf, 72, throttle);
	_mav_put_uint8_t(buf, 74, can_run1);
	_mav_put_uint8_t(buf, 75, can_run2);
	_mav_put_uint8_t(buf, 76, can_run3);
	_mav_put_uint8_t(buf, 77, can_run4);
	_mav_put_uint8_t(buf, 78, angle_boost);
	_mav_put_uint8_t(buf, 79, last_shim);
	_mav_put_uint8_t(buf, 80, safe_x);
	_mav_put_uint8_t(buf, 81, safe_y);
	_mav_put_uint8_t(buf, 82, safe_z);

#if MAVLINK_CRC_EXTRA
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_SHIM_STATS, buf, MAVLINK_MSG_ID_SHIM_STATS_LEN, MAVLINK_MSG_ID_SHIM_STATS_CRC);
#else
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_SHIM_STATS, buf, MAVLINK_MSG_ID_SHIM_STATS_LEN);
#endif
#else
	mavlink_shim_stats_t packet;
	packet.x = x;
	packet.y = y;
	packet.z = z;
	packet.vx = vx;
	packet.vy = vy;
	packet.vz = vz;
	packet.A_proposed = A_proposed;
	packet.Roll_proposed = Roll_proposed;
	packet.Pitch_proposed = Pitch_proposed;
	packet.a = a;
	packet.roll = roll;
	packet.pitch = pitch;
	packet.AX_proposed = AX_proposed;
	packet.AY_proposed = AY_proposed;
	packet.AZ_proposed = AZ_proposed;
	packet.ax = ax;
	packet.ay = ay;
	packet.az = az;
	packet.throttle = throttle;
	packet.can_run1 = can_run1;
	packet.can_run2 = can_run2;
	packet.can_run3 = can_run3;
	packet.can_run4 = can_run4;
	packet.angle_boost = angle_boost;
	packet.last_shim = last_shim;
	packet.safe_x = safe_x;
	packet.safe_y = safe_y;
	packet.safe_z = safe_z;

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
static inline void mavlink_msg_shim_stats_send_buf(mavlink_message_t *msgbuf, mavlink_channel_t chan,  uint8_t can_run1, uint8_t can_run2, uint8_t can_run3, uint8_t can_run4, float x, float y, float z, float vx, float vy, float vz, uint16_t throttle, uint8_t angle_boost, float A_proposed, float Roll_proposed, float Pitch_proposed, float a, float roll, float pitch, uint8_t last_shim, float AX_proposed, float AY_proposed, float AZ_proposed, float ax, float ay, float az, uint8_t safe_x, uint8_t safe_y, uint8_t safe_z)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
	char *buf = (char *)msgbuf;
	_mav_put_float(buf, 0, x);
	_mav_put_float(buf, 4, y);
	_mav_put_float(buf, 8, z);
	_mav_put_float(buf, 12, vx);
	_mav_put_float(buf, 16, vy);
	_mav_put_float(buf, 20, vz);
	_mav_put_float(buf, 24, A_proposed);
	_mav_put_float(buf, 28, Roll_proposed);
	_mav_put_float(buf, 32, Pitch_proposed);
	_mav_put_float(buf, 36, a);
	_mav_put_float(buf, 40, roll);
	_mav_put_float(buf, 44, pitch);
	_mav_put_float(buf, 48, AX_proposed);
	_mav_put_float(buf, 52, AY_proposed);
	_mav_put_float(buf, 56, AZ_proposed);
	_mav_put_float(buf, 60, ax);
	_mav_put_float(buf, 64, ay);
	_mav_put_float(buf, 68, az);
	_mav_put_uint16_t(buf, 72, throttle);
	_mav_put_uint8_t(buf, 74, can_run1);
	_mav_put_uint8_t(buf, 75, can_run2);
	_mav_put_uint8_t(buf, 76, can_run3);
	_mav_put_uint8_t(buf, 77, can_run4);
	_mav_put_uint8_t(buf, 78, angle_boost);
	_mav_put_uint8_t(buf, 79, last_shim);
	_mav_put_uint8_t(buf, 80, safe_x);
	_mav_put_uint8_t(buf, 81, safe_y);
	_mav_put_uint8_t(buf, 82, safe_z);

#if MAVLINK_CRC_EXTRA
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_SHIM_STATS, buf, MAVLINK_MSG_ID_SHIM_STATS_LEN, MAVLINK_MSG_ID_SHIM_STATS_CRC);
#else
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_SHIM_STATS, buf, MAVLINK_MSG_ID_SHIM_STATS_LEN);
#endif
#else
	mavlink_shim_stats_t *packet = (mavlink_shim_stats_t *)msgbuf;
	packet->x = x;
	packet->y = y;
	packet->z = z;
	packet->vx = vx;
	packet->vy = vy;
	packet->vz = vz;
	packet->A_proposed = A_proposed;
	packet->Roll_proposed = Roll_proposed;
	packet->Pitch_proposed = Pitch_proposed;
	packet->a = a;
	packet->roll = roll;
	packet->pitch = pitch;
	packet->AX_proposed = AX_proposed;
	packet->AY_proposed = AY_proposed;
	packet->AZ_proposed = AZ_proposed;
	packet->ax = ax;
	packet->ay = ay;
	packet->az = az;
	packet->throttle = throttle;
	packet->can_run1 = can_run1;
	packet->can_run2 = can_run2;
	packet->can_run3 = can_run3;
	packet->can_run4 = can_run4;
	packet->angle_boost = angle_boost;
	packet->last_shim = last_shim;
	packet->safe_x = safe_x;
	packet->safe_y = safe_y;
	packet->safe_z = safe_z;

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
	return _MAV_RETURN_uint8_t(msg,  74);
}

/**
 * @brief Get field can_run2 from shim_stats message
 *
 * @return 1 if shim 2 can run, 0 otherwise
 */
static inline uint8_t mavlink_msg_shim_stats_get_can_run2(const mavlink_message_t* msg)
{
	return _MAV_RETURN_uint8_t(msg,  75);
}

/**
 * @brief Get field can_run3 from shim_stats message
 *
 * @return 1 if shim 3 can run, 0 otherwise
 */
static inline uint8_t mavlink_msg_shim_stats_get_can_run3(const mavlink_message_t* msg)
{
	return _MAV_RETURN_uint8_t(msg,  76);
}

/**
 * @brief Get field can_run4 from shim_stats message
 *
 * @return 1 if shim 4 can run, 0 otherwise
 */
static inline uint8_t mavlink_msg_shim_stats_get_can_run4(const mavlink_message_t* msg)
{
	return _MAV_RETURN_uint8_t(msg,  77);
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
 * @return The value of y, relative to starting position.
 */
static inline float mavlink_msg_shim_stats_get_y(const mavlink_message_t* msg)
{
	return _MAV_RETURN_float(msg,  4);
}

/**
 * @brief Get field z from shim_stats message
 *
 * @return The height, relative to starting position.
 */
static inline float mavlink_msg_shim_stats_get_z(const mavlink_message_t* msg)
{
	return _MAV_RETURN_float(msg,  8);
}

/**
 * @brief Get field vx from shim_stats message
 *
 * @return The velocity in the x direction.
 */
static inline float mavlink_msg_shim_stats_get_vx(const mavlink_message_t* msg)
{
	return _MAV_RETURN_float(msg,  12);
}

/**
 * @brief Get field vy from shim_stats message
 *
 * @return The velocity in the y direction.
 */
static inline float mavlink_msg_shim_stats_get_vy(const mavlink_message_t* msg)
{
	return _MAV_RETURN_float(msg,  16);
}

/**
 * @brief Get field vz from shim_stats message
 *
 * @return The velocity in the vertical direction.
 */
static inline float mavlink_msg_shim_stats_get_vz(const mavlink_message_t* msg)
{
	return _MAV_RETURN_float(msg,  20);
}

/**
 * @brief Get field throttle from shim_stats message
 *
 * @return The proposed throttle received by the shim
 */
static inline uint16_t mavlink_msg_shim_stats_get_throttle(const mavlink_message_t* msg)
{
	return _MAV_RETURN_uint16_t(msg,  72);
}

/**
 * @brief Get field angle_boost from shim_stats message
 *
 * @return 1 if angle boost was requested, 0 otherwise
 */
static inline uint8_t mavlink_msg_shim_stats_get_angle_boost(const mavlink_message_t* msg)
{
	return _MAV_RETURN_uint8_t(msg,  78);
}

/**
 * @brief Get field A_proposed from shim_stats message
 *
 * @return The proposed acceleration, normal to the quadcopter.
 */
static inline float mavlink_msg_shim_stats_get_A_proposed(const mavlink_message_t* msg)
{
	return _MAV_RETURN_float(msg,  24);
}

/**
 * @brief Get field Roll_proposed from shim_stats message
 *
 * @return The proposed roll angle
 */
static inline float mavlink_msg_shim_stats_get_Roll_proposed(const mavlink_message_t* msg)
{
	return _MAV_RETURN_float(msg,  28);
}

/**
 * @brief Get field Pitch_proposed from shim_stats message
 *
 * @return The proposed pitch angle
 */
static inline float mavlink_msg_shim_stats_get_Pitch_proposed(const mavlink_message_t* msg)
{
	return _MAV_RETURN_float(msg,  32);
}

/**
 * @brief Get field a from shim_stats message
 *
 * @return The issued acceleration, normal to the quadcopter.
 */
static inline float mavlink_msg_shim_stats_get_a(const mavlink_message_t* msg)
{
	return _MAV_RETURN_float(msg,  36);
}

/**
 * @brief Get field roll from shim_stats message
 *
 * @return The issued roll angle
 */
static inline float mavlink_msg_shim_stats_get_roll(const mavlink_message_t* msg)
{
	return _MAV_RETURN_float(msg,  40);
}

/**
 * @brief Get field pitch from shim_stats message
 *
 * @return The issued pitch angle
 */
static inline float mavlink_msg_shim_stats_get_pitch(const mavlink_message_t* msg)
{
	return _MAV_RETURN_float(msg,  44);
}

/**
 * @brief Get field last_shim from shim_stats message
 *
 * @return The last shim to run
 */
static inline uint8_t mavlink_msg_shim_stats_get_last_shim(const mavlink_message_t* msg)
{
	return _MAV_RETURN_uint8_t(msg,  79);
}

/**
 * @brief Get field AX_proposed from shim_stats message
 *
 * @return The proposed AX
 */
static inline float mavlink_msg_shim_stats_get_AX_proposed(const mavlink_message_t* msg)
{
	return _MAV_RETURN_float(msg,  48);
}

/**
 * @brief Get field AY_proposed from shim_stats message
 *
 * @return The proposed AY
 */
static inline float mavlink_msg_shim_stats_get_AY_proposed(const mavlink_message_t* msg)
{
	return _MAV_RETURN_float(msg,  52);
}

/**
 * @brief Get field AZ_proposed from shim_stats message
 *
 * @return The proposed AZ
 */
static inline float mavlink_msg_shim_stats_get_AZ_proposed(const mavlink_message_t* msg)
{
	return _MAV_RETURN_float(msg,  56);
}

/**
 * @brief Get field ax from shim_stats message
 *
 * @return The actual ax
 */
static inline float mavlink_msg_shim_stats_get_ax(const mavlink_message_t* msg)
{
	return _MAV_RETURN_float(msg,  60);
}

/**
 * @brief Get field ay from shim_stats message
 *
 * @return The actual ay
 */
static inline float mavlink_msg_shim_stats_get_ay(const mavlink_message_t* msg)
{
	return _MAV_RETURN_float(msg,  64);
}

/**
 * @brief Get field az from shim_stats message
 *
 * @return The actual az
 */
static inline float mavlink_msg_shim_stats_get_az(const mavlink_message_t* msg)
{
	return _MAV_RETURN_float(msg,  68);
}

/**
 * @brief Get field safe_x from shim_stats message
 *
 * @return 1 if safe in x dimension, 0 otherwise
 */
static inline uint8_t mavlink_msg_shim_stats_get_safe_x(const mavlink_message_t* msg)
{
	return _MAV_RETURN_uint8_t(msg,  80);
}

/**
 * @brief Get field safe_y from shim_stats message
 *
 * @return 1 if safe in y dimension, 0 otherwise
 */
static inline uint8_t mavlink_msg_shim_stats_get_safe_y(const mavlink_message_t* msg)
{
	return _MAV_RETURN_uint8_t(msg,  81);
}

/**
 * @brief Get field safe_z from shim_stats message
 *
 * @return 1 if safe in z dimension, 0 otherwise
 */
static inline uint8_t mavlink_msg_shim_stats_get_safe_z(const mavlink_message_t* msg)
{
	return _MAV_RETURN_uint8_t(msg,  82);
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
	shim_stats->z = mavlink_msg_shim_stats_get_z(msg);
	shim_stats->vx = mavlink_msg_shim_stats_get_vx(msg);
	shim_stats->vy = mavlink_msg_shim_stats_get_vy(msg);
	shim_stats->vz = mavlink_msg_shim_stats_get_vz(msg);
	shim_stats->A_proposed = mavlink_msg_shim_stats_get_A_proposed(msg);
	shim_stats->Roll_proposed = mavlink_msg_shim_stats_get_Roll_proposed(msg);
	shim_stats->Pitch_proposed = mavlink_msg_shim_stats_get_Pitch_proposed(msg);
	shim_stats->a = mavlink_msg_shim_stats_get_a(msg);
	shim_stats->roll = mavlink_msg_shim_stats_get_roll(msg);
	shim_stats->pitch = mavlink_msg_shim_stats_get_pitch(msg);
	shim_stats->AX_proposed = mavlink_msg_shim_stats_get_AX_proposed(msg);
	shim_stats->AY_proposed = mavlink_msg_shim_stats_get_AY_proposed(msg);
	shim_stats->AZ_proposed = mavlink_msg_shim_stats_get_AZ_proposed(msg);
	shim_stats->ax = mavlink_msg_shim_stats_get_ax(msg);
	shim_stats->ay = mavlink_msg_shim_stats_get_ay(msg);
	shim_stats->az = mavlink_msg_shim_stats_get_az(msg);
	shim_stats->throttle = mavlink_msg_shim_stats_get_throttle(msg);
	shim_stats->can_run1 = mavlink_msg_shim_stats_get_can_run1(msg);
	shim_stats->can_run2 = mavlink_msg_shim_stats_get_can_run2(msg);
	shim_stats->can_run3 = mavlink_msg_shim_stats_get_can_run3(msg);
	shim_stats->can_run4 = mavlink_msg_shim_stats_get_can_run4(msg);
	shim_stats->angle_boost = mavlink_msg_shim_stats_get_angle_boost(msg);
	shim_stats->last_shim = mavlink_msg_shim_stats_get_last_shim(msg);
	shim_stats->safe_x = mavlink_msg_shim_stats_get_safe_x(msg);
	shim_stats->safe_y = mavlink_msg_shim_stats_get_safe_y(msg);
	shim_stats->safe_z = mavlink_msg_shim_stats_get_safe_z(msg);
#else
	memcpy(shim_stats, _MAV_PAYLOAD(msg), MAVLINK_MSG_ID_SHIM_STATS_LEN);
#endif
}
