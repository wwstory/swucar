// Auto-generated. Do not edit!

// (in-package image_detect.msg)


"use strict";

const _serializer = _ros_msg_utils.Serialize;
const _arraySerializer = _serializer.Array;
const _deserializer = _ros_msg_utils.Deserialize;
const _arrayDeserializer = _deserializer.Array;
const _finder = _ros_msg_utils.Find;
const _getByteLength = _ros_msg_utils.getByteLength;

//-----------------------------------------------------------

class Detection {
  constructor(initObj={}) {
    if (initObj === null) {
      // initObj === null is a special case for deserialization where we don't initialize fields
      this.pos = null;
      this.classes = null;
      this.scores = null;
    }
    else {
      if (initObj.hasOwnProperty('pos')) {
        this.pos = initObj.pos
      }
      else {
        this.pos = [];
      }
      if (initObj.hasOwnProperty('classes')) {
        this.classes = initObj.classes
      }
      else {
        this.classes = [];
      }
      if (initObj.hasOwnProperty('scores')) {
        this.scores = initObj.scores
      }
      else {
        this.scores = [];
      }
    }
  }

  static serialize(obj, buffer, bufferOffset) {
    // Serializes a message object of type Detection
    // Serialize message field [pos]
    bufferOffset = _arraySerializer.float32(obj.pos, buffer, bufferOffset, null);
    // Serialize message field [classes]
    bufferOffset = _arraySerializer.uint32(obj.classes, buffer, bufferOffset, null);
    // Serialize message field [scores]
    bufferOffset = _arraySerializer.float32(obj.scores, buffer, bufferOffset, null);
    return bufferOffset;
  }

  static deserialize(buffer, bufferOffset=[0]) {
    //deserializes a message object of type Detection
    let len;
    let data = new Detection(null);
    // Deserialize message field [pos]
    data.pos = _arrayDeserializer.float32(buffer, bufferOffset, null)
    // Deserialize message field [classes]
    data.classes = _arrayDeserializer.uint32(buffer, bufferOffset, null)
    // Deserialize message field [scores]
    data.scores = _arrayDeserializer.float32(buffer, bufferOffset, null)
    return data;
  }

  static getMessageSize(object) {
    let length = 0;
    length += 4 * object.pos.length;
    length += 4 * object.classes.length;
    length += 4 * object.scores.length;
    return length + 12;
  }

  static datatype() {
    // Returns string type for a message object
    return 'image_detect/Detection';
  }

  static md5sum() {
    //Returns md5sum for a message object
    return '35145143adb0a6def78c300efff37377';
  }

  static messageDefinition() {
    // Returns full string definition for message
    return `
    float32[] pos       # 百分比位置 [x1, y1, x2, y2, _x1, _y1, _x2, _y2 ...]
    uint32[] classes    # 类别
    float32[] scores    # 分类自信值
    `;
  }

  static Resolve(msg) {
    // deep-construct a valid message object instance of whatever was passed in
    if (typeof msg !== 'object' || msg === null) {
      msg = {};
    }
    const resolved = new Detection(null);
    if (msg.pos !== undefined) {
      resolved.pos = msg.pos;
    }
    else {
      resolved.pos = []
    }

    if (msg.classes !== undefined) {
      resolved.classes = msg.classes;
    }
    else {
      resolved.classes = []
    }

    if (msg.scores !== undefined) {
      resolved.scores = msg.scores;
    }
    else {
      resolved.scores = []
    }

    return resolved;
    }
};

module.exports = Detection;
