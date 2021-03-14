// Auto-generated. Do not edit!

// (in-package hrwros_msgs.msg)


"use strict";

const _serializer = _ros_msg_utils.Serialize;
const _arraySerializer = _serializer.Array;
const _deserializer = _ros_msg_utils.Deserialize;
const _arrayDeserializer = _deserializer.Array;
const _finder = _ros_msg_utils.Find;
const _getByteLength = _ros_msg_utils.getByteLength;

//-----------------------------------------------------------

class Points_arrays {
  constructor(initObj={}) {
    if (initObj === null) {
      // initObj === null is a special case for deserialization where we don't initialize fields
      this.x_coordinates = null;
      this.y_coordinates = null;
    }
    else {
      if (initObj.hasOwnProperty('x_coordinates')) {
        this.x_coordinates = initObj.x_coordinates
      }
      else {
        this.x_coordinates = [];
      }
      if (initObj.hasOwnProperty('y_coordinates')) {
        this.y_coordinates = initObj.y_coordinates
      }
      else {
        this.y_coordinates = [];
      }
    }
  }

  static serialize(obj, buffer, bufferOffset) {
    // Serializes a message object of type Points_arrays
    // Serialize message field [x_coordinates]
    bufferOffset = _arraySerializer.uint8(obj.x_coordinates, buffer, bufferOffset, null);
    // Serialize message field [y_coordinates]
    bufferOffset = _arraySerializer.uint8(obj.y_coordinates, buffer, bufferOffset, null);
    return bufferOffset;
  }

  static deserialize(buffer, bufferOffset=[0]) {
    //deserializes a message object of type Points_arrays
    let len;
    let data = new Points_arrays(null);
    // Deserialize message field [x_coordinates]
    data.x_coordinates = _arrayDeserializer.uint8(buffer, bufferOffset, null)
    // Deserialize message field [y_coordinates]
    data.y_coordinates = _arrayDeserializer.uint8(buffer, bufferOffset, null)
    return data;
  }

  static getMessageSize(object) {
    let length = 0;
    length += object.x_coordinates.length;
    length += object.y_coordinates.length;
    return length + 8;
  }

  static datatype() {
    // Returns string type for a message object
    return 'hrwros_msgs/Points_arrays';
  }

  static md5sum() {
    //Returns md5sum for a message object
    return '3b76707f47a52d893eafcf2404ef94bc';
  }

  static messageDefinition() {
    // Returns full string definition for message
    return `
    uint8[] x_coordinates
    uint8[] y_coordinates
    
    `;
  }

  static Resolve(msg) {
    // deep-construct a valid message object instance of whatever was passed in
    if (typeof msg !== 'object' || msg === null) {
      msg = {};
    }
    const resolved = new Points_arrays(null);
    if (msg.x_coordinates !== undefined) {
      resolved.x_coordinates = msg.x_coordinates;
    }
    else {
      resolved.x_coordinates = []
    }

    if (msg.y_coordinates !== undefined) {
      resolved.y_coordinates = msg.y_coordinates;
    }
    else {
      resolved.y_coordinates = []
    }

    return resolved;
    }
};

module.exports = Points_arrays;
