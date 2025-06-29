// MESSAGE SET_CAM_SHUTTER PACKING

#define MAVLINK_MSG_ID_SET_CAM_SHUTTER 151

typedef struct __mavlink_set_cam_shutter_t
{
 uint8_t cam_no; ///< Camera id
 uint8_t cam_mode; ///< Camera mode: 0 = auto, 1 = manual
 uint8_t trigger_pin; ///< Trigger pin, 0-3 for PtGrey FireFly
 uint16_t interval; ///< Shutter interval, in microseconds
 uint16_t exposure; ///< Exposure time, in microseconds
 float gain; ///< Camera gain
} mavlink_set_cam_shutter_t;

#define MAVLINK_MSG_ID_SET_CAM_SHUTTER_LEN 11
#define MAVLINK_MSG_ID_151_LEN 11



#define MAVLINK_MESSAGE_INFO_SET_CAM_SHUTTER { \
	"SET_CAM_SHUTTER", \
	6, \
	{  { "cam_no", NULL, MAVLINK_TYPE_UINT8_T, 0, 0, offsetof(mavlink_set_cam_shutter_t, cam_no) }, \
         { "cam_mode", NULL, MAVLINK_TYPE_UINT8_T, 0, 1, offsetof(mavlink_set_cam_shutter_t, cam_mode) }, \
         { "trigger_pin", NULL, MAVLINK_TYPE_UINT8_T, 0, 2, offsetof(mavlink_set_cam_shutter_t, trigger_pin) }, \
         { "interval", NULL, MAVLINK_TYPE_UINT16_T, 0, 3, offsetof(mavlink_set_cam_shutter_t, interval) }, \
         { "exposure", NULL, MAVLINK_TYPE_UINT16_T, 0, 5, offsetof(mavlink_set_cam_shutter_t, exposure) }, \
         { "gain", NULL, MAVLINK_TYPE_FLOAT, 0, 7, offsetof(mavlink_set_cam_shutter_t, gain) }, \
         } \
}


/**
 * @brief Pack a set_cam_shutter message
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param msg The MAVLink message to compress the data into
 *
 * @param cam_no Camera id
 * @param cam_mode Camera mode: 0 = auto, 1 = manual
 * @param trigger_pin Trigger pin, 0-3 for PtGrey FireFly
 * @param interval Shutter interval, in microseconds
 * @param exposure Exposure time, in microseconds
 * @param gain Camera gain
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t mavlink_msg_set_cam_shutter_pack(uint8_t system_id, uint8_t component_id, mavlink_message_t* msg,
						       uint8_t cam_no, uint8_t cam_mode, uint8_t trigger_pin, uint16_t interval, uint16_t exposure, float gain)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
	char buf[11];
	_mav_put_uint8_t(buf, 0, cam_no);
	_mav_put_uint8_t(buf, 1, cam_mode);
	_mav_put_uint8_t(buf, 2, trigger_pin);
	_mav_put_uint16_t(buf, 3, interval);
	_mav_put_uint16_t(buf, 5, exposure);
	_mav_put_float(buf, 7, gain);

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), buf, 11);
#else
	mavlink_set_cam_shutter_t packet;
	packet.cam_no = cam_no;
	packet.cam_mode = cam_mode;
	packet.trigger_pin = trigger_pin;
	packet.interval = interval;
	packet.exposure = exposure;
	packet.gain = gain;

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), &packet, 11);
#endif

	msg->msgid = MAVLINK_MSG_ID_SET_CAM_SHUTTER;
	return mavlink_finalize_message(msg, system_id, component_id, 11);
}

/**
 * @brief Pack a set_cam_shutter message on a channel
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param chan The MAVLink channel this message was sent over
 * @param msg The MAVLink message to compress the data into
 * @param cam_no Camera id
 * @param cam_mode Camera mode: 0 = auto, 1 = manual
 * @param trigger_pin Trigger pin, 0-3 for PtGrey FireFly
 * @param interval Shutter interval, in microseconds
 * @param exposure Exposure time, in microseconds
 * @param gain Camera gain
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t mavlink_msg_set_cam_shutter_pack_chan(uint8_t system_id, uint8_t component_id, uint8_t chan,
							   mavlink_message_t* msg,
						           uint8_t cam_no,uint8_t cam_mode,uint8_t trigger_pin,uint16_t interval,uint16_t exposure,float gain)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
	char buf[11];
	_mav_put_uint8_t(buf, 0, cam_no);
	_mav_put_uint8_t(buf, 1, cam_mode);
	_mav_put_uint8_t(buf, 2, trigger_pin);
	_mav_put_uint16_t(buf, 3, interval);
	_mav_put_uint16_t(buf, 5, exposure);
	_mav_put_float(buf, 7, gain);

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), buf, 11);
#else
	mavlink_set_cam_shutter_t packet;
	packet.cam_no = cam_no;
	packet.cam_mode = cam_mode;
	packet.trigger_pin = trigger_pin;
	packet.interval = interval;
	packet.exposure = exposure;
	packet.gain = gain;

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), &packet, 11);
#endif

	msg->msgid = MAVLINK_MSG_ID_SET_CAM_SHUTTER;
	return mavlink_finalize_message_chan(msg, system_id, component_id, chan, 11);
}

/**
 * @brief Encode a set_cam_shutter struct into a message
 *
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param msg The MAVLink message to compress the data into
 * @param set_cam_shutter C-struct to read the message contents from
 */
static inline uint16_t mavlink_msg_set_cam_shutter_encode(uint8_t system_id, uint8_t component_id, mavlink_message_t* msg, const mavlink_set_cam_shutter_t* set_cam_shutter)
{
	return mavlink_msg_set_cam_shutter_pack(system_id, component_id, msg, set_cam_shutter->cam_no, set_cam_shutter->cam_mode, set_cam_shutter->trigger_pin, set_cam_shutter->interval, set_cam_shutter->exposure, set_cam_shutter->gain);
}

/**
 * @brief Send a set_cam_shutter message
 * @param chan MAVLink channel to send the message
 *
 * @param cam_no Camera id
 * @param cam_mode Camera mode: 0 = auto, 1 = manual
 * @param trigger_pin Trigger pin, 0-3 for PtGrey FireFly
 * @param interval Shutter interval, in microseconds
 * @param exposure Exposure time, in microseconds
 * @param gain Camera gain
 */
#ifdef MAVLINK_USE_CONVENIENCE_FUNCTIONS

static inline void mavlink_msg_set_cam_shutter_send(mavlink_channel_t chan, uint8_t cam_no, uint8_t cam_mode, uint8_t trigger_pin, uint16_t interval, uint16_t exposure, float gain)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
	char buf[11];
	_mav_put_uint8_t(buf, 0, cam_no);
	_mav_put_uint8_t(buf, 1, cam_mode);
	_mav_put_uint8_t(buf, 2, trigger_pin);
	_mav_put_uint16_t(buf, 3, interval);
	_mav_put_uint16_t(buf, 5, exposure);
	_mav_put_float(buf, 7, gain);

	_mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_SET_CAM_SHUTTER, buf, 11);
#else
	mavlink_set_cam_shutter_t packet;
	packet.cam_no = cam_no;
	packet.cam_mode = cam_mode;
	packet.trigger_pin = trigger_pin;
	packet.interval = interval;
	packet.exposure = exposure;
	packet.gain = gain;

	_mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_SET_CAM_SHUTTER, (const char *)&packet, 11);
#endif
}

#endif

// MESSAGE SET_CAM_SHUTTER UNPACKING


/**
 * @brief Get field cam_no from set_cam_shutter message
 *
 * @return Camera id
 */
static inline uint8_t mavlink_msg_set_cam_shutter_get_cam_no(const mavlink_message_t* msg)
{
	return _MAV_RETURN_uint8_t(msg,  0);
}

/**
 * @brief Get field cam_mode from set_cam_shutter message
 *
 * @return Camera mode: 0 = auto, 1 = manual
 */
static inline uint8_t mavlink_msg_set_cam_shutter_get_cam_mode(const mavlink_message_t* msg)
{
	return _MAV_RETURN_uint8_t(msg,  1);
}

/**
 * @brief Get field trigger_pin from set_cam_shutter message
 *
 * @return Trigger pin, 0-3 for PtGrey FireFly
 */
static inline uint8_t mavlink_msg_set_cam_shutter_get_trigger_pin(const mavlink_message_t* msg)
{
	return _MAV_RETURN_uint8_t(msg,  2);
}

/**
 * @brief Get field interval from set_cam_shutter message
 *
 * @return Shutter interval, in microseconds
 */
static inline uint16_t mavlink_msg_set_cam_shutter_get_interval(const mavlink_message_t* msg)
{
	return _MAV_RETURN_uint16_t(msg,  3);
}

/**
 * @brief Get field exposure from set_cam_shutter message
 *
 * @return Exposure time, in microseconds
 */
static inline uint16_t mavlink_msg_set_cam_shutter_get_exposure(const mavlink_message_t* msg)
{
	return _MAV_RETURN_uint16_t(msg,  5);
}

/**
 * @brief Get field gain from set_cam_shutter message
 *
 * @return Camera gain
 */
static inline float mavlink_msg_set_cam_shutter_get_gain(const mavlink_message_t* msg)
{
	return _MAV_RETURN_float(msg,  7);
}

/**
 * @brief Decode a set_cam_shutter message into a struct
 *
 * @param msg The message to decode
 * @param set_cam_shutter C-struct to decode the message contents into
 */
static inline void mavlink_msg_set_cam_shutter_decode(const mavlink_message_t* msg, mavlink_set_cam_shutter_t* set_cam_shutter)
{
#if MAVLINK_NEED_BYTE_SWAP
	set_cam_shutter->cam_no = mavlink_msg_set_cam_shutter_get_cam_no(msg);
	set_cam_shutter->cam_mode = mavlink_msg_set_cam_shutter_get_cam_mode(msg);
	set_cam_shutter->trigger_pin = mavlink_msg_set_cam_shutter_get_trigger_pin(msg);
	set_cam_shutter->interval = mavlink_msg_set_cam_shutter_get_interval(msg);
	set_cam_shutter->exposure = mavlink_msg_set_cam_shutter_get_exposure(msg);
	set_cam_shutter->gain = mavlink_msg_set_cam_shutter_get_gain(msg);
#else
	memcpy(set_cam_shutter, _MAV_PAYLOAD(msg), 11);
#endif
}
