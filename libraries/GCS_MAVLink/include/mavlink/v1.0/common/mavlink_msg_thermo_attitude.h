// MESSAGE THERMO_ATTITUDE PACKING

#define MAVLINK_MSG_ID_THERMO_ATTITUDE 174

typedef struct __mavlink_thermo_attitude_t
{
 uint32_t time_boot_ms; ///< Timestamp (milliseconds since system boot)
 float thermo_roll; ///< Roll angle (rad)
 float thermo_pitch; ///< Pitch angle (rad)
} mavlink_thermo_attitude_t;

#define MAVLINK_MSG_ID_THERMO_ATTITUDE_LEN 12
#define MAVLINK_MSG_ID_154_LEN 12



#define MAVLINK_MESSAGE_INFO_THERMO_ATTITUDE { \
	"THERMO_ATTITUDE", \
	3, \
	{  { "time_boot_ms", NULL, MAVLINK_TYPE_UINT32_T, 0, 0, offsetof(mavlink_thermo_attitude_t, time_boot_ms) }, \
         { "thermo_roll", NULL, MAVLINK_TYPE_FLOAT, 0, 4, offsetof(mavlink_thermo_attitude_t, thermo_roll) }, \
         { "thermo_pitch", NULL, MAVLINK_TYPE_FLOAT, 0, 8, offsetof(mavlink_thermo_attitude_t, thermo_pitch) }, \
         } \
}


/**
 * @brief Pack a thermo_attitude message
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param msg The MAVLink message to compress the data into
 *
 * @param time_boot_ms Timestamp (milliseconds since system boot)
 * @param thermo_roll Roll angle (rad)
 * @param thermo_pitch Pitch angle (rad)
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t mavlink_msg_thermo_attitude_pack(uint8_t system_id, uint8_t component_id, mavlink_message_t* msg,
						       uint32_t time_boot_ms, float thermo_roll, float thermo_pitch)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
	char buf[12];
	_mav_put_uint32_t(buf, 0, time_boot_ms);
	_mav_put_float(buf, 4, thermo_roll);
	_mav_put_float(buf, 8, thermo_pitch);

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), buf, 12);
#else
	mavlink_thermo_attitude_t packet;
	packet.time_boot_ms = time_boot_ms;
	packet.thermo_roll = thermo_roll;
	packet.thermo_pitch = thermo_pitch;

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), &packet, 12);
#endif

	msg->msgid = MAVLINK_MSG_ID_THERMO_ATTITUDE;
	return mavlink_finalize_message(msg, system_id, component_id, 12, 162);
}

/**
 * @brief Pack a thermo_attitude message on a channel
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param chan The MAVLink channel this message was sent over
 * @param msg The MAVLink message to compress the data into
 * @param time_boot_ms Timestamp (milliseconds since system boot)
 * @param thermo_roll Roll angle (rad)
 * @param thermo_pitch Pitch angle (rad)
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t mavlink_msg_thermo_attitude_pack_chan(uint8_t system_id, uint8_t component_id, uint8_t chan,
							   mavlink_message_t* msg,
						           uint32_t time_boot_ms,float thermo_roll,float thermo_pitch)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
	char buf[12];
	_mav_put_uint32_t(buf, 0, time_boot_ms);
	_mav_put_float(buf, 4, thermo_roll);
	_mav_put_float(buf, 8, thermo_pitch);

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), buf, 12);
#else
	mavlink_thermo_attitude_t packet;
	packet.time_boot_ms = time_boot_ms;
	packet.thermo_roll = thermo_roll;
	packet.thermo_pitch = thermo_pitch;

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), &packet, 12);
#endif

	msg->msgid = MAVLINK_MSG_ID_THERMO_ATTITUDE;
	return mavlink_finalize_message_chan(msg, system_id, component_id, chan, 12, 162);
}

/**
 * @brief Encode a thermo_attitude struct into a message
 *
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param msg The MAVLink message to compress the data into
 * @param thermo_attitude C-struct to read the message contents from
 */
static inline uint16_t mavlink_msg_thermo_attitude_encode(uint8_t system_id, uint8_t component_id, mavlink_message_t* msg, const mavlink_thermo_attitude_t* thermo_attitude)
{
	return mavlink_msg_thermo_attitude_pack(system_id, component_id, msg, thermo_attitude->time_boot_ms, thermo_attitude->thermo_roll, thermo_attitude->thermo_pitch);
}

/**
 * @brief Send a thermo_attitude message
 * @param chan MAVLink channel to send the message
 *
 * @param time_boot_ms Timestamp (milliseconds since system boot)
 * @param thermo_roll Roll angle (rad)
 * @param thermo_pitch Pitch angle (rad)
 */
#ifdef MAVLINK_USE_CONVENIENCE_FUNCTIONS

static inline void mavlink_msg_thermo_attitude_send(mavlink_channel_t chan, uint32_t time_boot_ms, float thermo_roll, float thermo_pitch)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
	char buf[12];
	_mav_put_uint32_t(buf, 0, time_boot_ms);
	_mav_put_float(buf, 4, thermo_roll);
	_mav_put_float(buf, 8, thermo_pitch);

	_mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_THERMO_ATTITUDE, buf, 12, 162);
#else
	mavlink_thermo_attitude_t packet;
	packet.time_boot_ms = time_boot_ms;
	packet.thermo_roll = thermo_roll;
	packet.thermo_pitch = thermo_pitch;

	_mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_THERMO_ATTITUDE, (const char *)&packet, 12, 162);
#endif
}

#endif

// MESSAGE THERMO_ATTITUDE UNPACKING


/**
 * @brief Get field time_boot_ms from thermo_attitude message
 *
 * @return Timestamp (milliseconds since system boot)
 */
static inline uint32_t mavlink_msg_thermo_attitude_get_time_boot_ms(const mavlink_message_t* msg)
{
	return _MAV_RETURN_uint32_t(msg,  0);
}

/**
 * @brief Get field thermo_roll from thermo_attitude message
 *
 * @return Roll angle (rad)
 */
static inline float mavlink_msg_thermo_attitude_get_thermo_roll(const mavlink_message_t* msg)
{
	return _MAV_RETURN_float(msg,  4);
}

/**
 * @brief Get field thermo_pitch from thermo_attitude message
 *
 * @return Pitch angle (rad)
 */
static inline float mavlink_msg_thermo_attitude_get_thermo_pitch(const mavlink_message_t* msg)
{
	return _MAV_RETURN_float(msg,  8);
}

/**
 * @brief Decode a thermo_attitude message into a struct
 *
 * @param msg The message to decode
 * @param thermo_attitude C-struct to decode the message contents into
 */
static inline void mavlink_msg_thermo_attitude_decode(const mavlink_message_t* msg, mavlink_thermo_attitude_t* thermo_attitude)
{
#if MAVLINK_NEED_BYTE_SWAP
	thermo_attitude->time_boot_ms = mavlink_msg_thermo_attitude_get_time_boot_ms(msg);
	thermo_attitude->thermo_roll = mavlink_msg_thermo_attitude_get_thermo_roll(msg);
	thermo_attitude->thermo_pitch = mavlink_msg_thermo_attitude_get_thermo_pitch(msg);
#else
	memcpy(thermo_attitude, _MAV_PAYLOAD(msg), 12);
#endif
}
