// MESSAGE THERMOPILE_ATTITUDE PACKING

#define MAVLINK_MSG_ID_THERMOPILE_ATTITUDE 174

typedef struct __mavlink_thermopile_attitude_t
{
 uint32_t time_boot_ms; ///< Timestamp (milliseconds since system boot)
 float roll; ///< Euler angle roll from thermopile sensors in radians. Positive for right roll
 float pitch; ///< Euler angle pitch from thermopile sensors in radians. Posititive for pitch up
} mavlink_thermopile_attitude_t;

#define MAVLINK_MSG_ID_THERMOPILE_ATTITUDE_LEN 12
#define MAVLINK_MSG_ID_174_LEN 12

#define MAVLINK_MSG_ID_THERMOPILE_ATTITUDE_CRC 60
#define MAVLINK_MSG_ID_174_CRC 60



#define MAVLINK_MESSAGE_INFO_THERMOPILE_ATTITUDE { \
	"THERMOPILE_ATTITUDE", \
	3, \
	{  { "time_boot_ms", NULL, MAVLINK_TYPE_UINT32_T, 0, 0, offsetof(mavlink_thermopile_attitude_t, time_boot_ms) }, \
         { "roll", NULL, MAVLINK_TYPE_FLOAT, 0, 4, offsetof(mavlink_thermopile_attitude_t, roll) }, \
         { "pitch", NULL, MAVLINK_TYPE_FLOAT, 0, 8, offsetof(mavlink_thermopile_attitude_t, pitch) }, \
         } \
}


/**
 * @brief Pack a thermopile_attitude message
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param msg The MAVLink message to compress the data into
 *
 * @param time_boot_ms Timestamp (milliseconds since system boot)
 * @param roll Euler angle roll from thermopile sensors in radians. Positive for right roll
 * @param pitch Euler angle pitch from thermopile sensors in radians. Posititive for pitch up
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t mavlink_msg_thermopile_attitude_pack(uint8_t system_id, uint8_t component_id, mavlink_message_t* msg,
						       uint32_t time_boot_ms, float roll, float pitch)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
	char buf[MAVLINK_MSG_ID_THERMOPILE_ATTITUDE_LEN];
	_mav_put_uint32_t(buf, 0, time_boot_ms);
	_mav_put_float(buf, 4, roll);
	_mav_put_float(buf, 8, pitch);

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), buf, MAVLINK_MSG_ID_THERMOPILE_ATTITUDE_LEN);
#else
	mavlink_thermopile_attitude_t packet;
	packet.time_boot_ms = time_boot_ms;
	packet.roll = roll;
	packet.pitch = pitch;

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), &packet, MAVLINK_MSG_ID_THERMOPILE_ATTITUDE_LEN);
#endif

	msg->msgid = MAVLINK_MSG_ID_THERMOPILE_ATTITUDE;
#if MAVLINK_CRC_EXTRA
    return mavlink_finalize_message(msg, system_id, component_id, MAVLINK_MSG_ID_THERMOPILE_ATTITUDE_LEN, MAVLINK_MSG_ID_THERMOPILE_ATTITUDE_CRC);
#else
    return mavlink_finalize_message(msg, system_id, component_id, MAVLINK_MSG_ID_THERMOPILE_ATTITUDE_LEN);
#endif
}

/**
 * @brief Pack a thermopile_attitude message on a channel
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param chan The MAVLink channel this message was sent over
 * @param msg The MAVLink message to compress the data into
 * @param time_boot_ms Timestamp (milliseconds since system boot)
 * @param roll Euler angle roll from thermopile sensors in radians. Positive for right roll
 * @param pitch Euler angle pitch from thermopile sensors in radians. Posititive for pitch up
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t mavlink_msg_thermopile_attitude_pack_chan(uint8_t system_id, uint8_t component_id, uint8_t chan,
							   mavlink_message_t* msg,
						           uint32_t time_boot_ms,float roll,float pitch)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
	char buf[MAVLINK_MSG_ID_THERMOPILE_ATTITUDE_LEN];
	_mav_put_uint32_t(buf, 0, time_boot_ms);
	_mav_put_float(buf, 4, roll);
	_mav_put_float(buf, 8, pitch);

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), buf, MAVLINK_MSG_ID_THERMOPILE_ATTITUDE_LEN);
#else
	mavlink_thermopile_attitude_t packet;
	packet.time_boot_ms = time_boot_ms;
	packet.roll = roll;
	packet.pitch = pitch;

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), &packet, MAVLINK_MSG_ID_THERMOPILE_ATTITUDE_LEN);
#endif

	msg->msgid = MAVLINK_MSG_ID_THERMOPILE_ATTITUDE;
#if MAVLINK_CRC_EXTRA
    return mavlink_finalize_message_chan(msg, system_id, component_id, chan, MAVLINK_MSG_ID_THERMOPILE_ATTITUDE_LEN, MAVLINK_MSG_ID_THERMOPILE_ATTITUDE_CRC);
#else
    return mavlink_finalize_message_chan(msg, system_id, component_id, chan, MAVLINK_MSG_ID_THERMOPILE_ATTITUDE_LEN);
#endif
}

/**
 * @brief Encode a thermopile_attitude struct into a message
 *
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param msg The MAVLink message to compress the data into
 * @param thermopile_attitude C-struct to read the message contents from
 */
static inline uint16_t mavlink_msg_thermopile_attitude_encode(uint8_t system_id, uint8_t component_id, mavlink_message_t* msg, const mavlink_thermopile_attitude_t* thermopile_attitude)
{
	return mavlink_msg_thermopile_attitude_pack(system_id, component_id, msg, thermopile_attitude->time_boot_ms, thermopile_attitude->roll, thermopile_attitude->pitch);
}

/**
 * @brief Send a thermopile_attitude message
 * @param chan MAVLink channel to send the message
 *
 * @param time_boot_ms Timestamp (milliseconds since system boot)
 * @param roll Euler angle roll from thermopile sensors in radians. Positive for right roll
 * @param pitch Euler angle pitch from thermopile sensors in radians. Posititive for pitch up
 */
#ifdef MAVLINK_USE_CONVENIENCE_FUNCTIONS

static inline void mavlink_msg_thermopile_attitude_send(mavlink_channel_t chan, uint32_t time_boot_ms, float roll, float pitch)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
	char buf[MAVLINK_MSG_ID_THERMOPILE_ATTITUDE_LEN];
	_mav_put_uint32_t(buf, 0, time_boot_ms);
	_mav_put_float(buf, 4, roll);
	_mav_put_float(buf, 8, pitch);

#if MAVLINK_CRC_EXTRA
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_THERMOPILE_ATTITUDE, buf, MAVLINK_MSG_ID_THERMOPILE_ATTITUDE_LEN, MAVLINK_MSG_ID_THERMOPILE_ATTITUDE_CRC);
#else
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_THERMOPILE_ATTITUDE, buf, MAVLINK_MSG_ID_THERMOPILE_ATTITUDE_LEN);
#endif
#else
	mavlink_thermopile_attitude_t packet;
	packet.time_boot_ms = time_boot_ms;
	packet.roll = roll;
	packet.pitch = pitch;

#if MAVLINK_CRC_EXTRA
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_THERMOPILE_ATTITUDE, (const char *)&packet, MAVLINK_MSG_ID_THERMOPILE_ATTITUDE_LEN, MAVLINK_MSG_ID_THERMOPILE_ATTITUDE_CRC);
#else
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_THERMOPILE_ATTITUDE, (const char *)&packet, MAVLINK_MSG_ID_THERMOPILE_ATTITUDE_LEN);
#endif
#endif
}

#endif

// MESSAGE THERMOPILE_ATTITUDE UNPACKING


/**
 * @brief Get field time_boot_ms from thermopile_attitude message
 *
 * @return Timestamp (milliseconds since system boot)
 */
static inline uint32_t mavlink_msg_thermopile_attitude_get_time_boot_ms(const mavlink_message_t* msg)
{
	return _MAV_RETURN_uint32_t(msg,  0);
}

/**
 * @brief Get field roll from thermopile_attitude message
 *
 * @return Euler angle roll from thermopile sensors in radians. Positive for right roll
 */
static inline float mavlink_msg_thermopile_attitude_get_roll(const mavlink_message_t* msg)
{
	return _MAV_RETURN_float(msg,  4);
}

/**
 * @brief Get field pitch from thermopile_attitude message
 *
 * @return Euler angle pitch from thermopile sensors in radians. Posititive for pitch up
 */
static inline float mavlink_msg_thermopile_attitude_get_pitch(const mavlink_message_t* msg)
{
	return _MAV_RETURN_float(msg,  8);
}

/**
 * @brief Decode a thermopile_attitude message into a struct
 *
 * @param msg The message to decode
 * @param thermopile_attitude C-struct to decode the message contents into
 */
static inline void mavlink_msg_thermopile_attitude_decode(const mavlink_message_t* msg, mavlink_thermopile_attitude_t* thermopile_attitude)
{
#if MAVLINK_NEED_BYTE_SWAP
	thermopile_attitude->time_boot_ms = mavlink_msg_thermopile_attitude_get_time_boot_ms(msg);
	thermopile_attitude->roll = mavlink_msg_thermopile_attitude_get_roll(msg);
	thermopile_attitude->pitch = mavlink_msg_thermopile_attitude_get_pitch(msg);
#else
	memcpy(thermopile_attitude, _MAV_PAYLOAD(msg), MAVLINK_MSG_ID_THERMOPILE_ATTITUDE_LEN);
#endif
}
