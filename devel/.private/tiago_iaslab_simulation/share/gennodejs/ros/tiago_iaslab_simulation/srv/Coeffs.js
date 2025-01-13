// Auto-generated. Do not edit!

// (in-package tiago_iaslab_simulation.srv)


"use strict";

const _serializer = _ros_msg_utils.Serialize;
const _arraySerializer = _serializer.Array;
const _deserializer = _ros_msg_utils.Deserialize;
const _arrayDeserializer = _deserializer.Array;
const _finder = _ros_msg_utils.Find;
const _getByteLength = _ros_msg_utils.getByteLength;

//-----------------------------------------------------------


//-----------------------------------------------------------

class CoeffsRequest {
  constructor(initObj={}) {
    if (initObj === null) {
      // initObj === null is a special case for deserialization where we don't initialize fields
      this.ready = null;
    }
    else {
      if (initObj.hasOwnProperty('ready')) {
        this.ready = initObj.ready
      }
      else {
        this.ready = false;
      }
    }
  }

  static serialize(obj, buffer, bufferOffset) {
    // Serializes a message object of type CoeffsRequest
    // Serialize message field [ready]
    bufferOffset = _serializer.bool(obj.ready, buffer, bufferOffset);
    return bufferOffset;
  }

  static deserialize(buffer, bufferOffset=[0]) {
    //deserializes a message object of type CoeffsRequest
    let len;
    let data = new CoeffsRequest(null);
    // Deserialize message field [ready]
    data.ready = _deserializer.bool(buffer, bufferOffset);
    return data;
  }

  static getMessageSize(object) {
    return 1;
  }

  static datatype() {
    // Returns string type for a service object
    return 'tiago_iaslab_simulation/CoeffsRequest';
  }

  static md5sum() {
    //Returns md5sum for a message object
    return '6f378c6311f9e6ccd2cd8c5b327003f1';
  }

  static messageDefinition() {
    // Returns full string definition for message
    return `
    bool ready          #if true get_straaightline_node sends the coefficients' values, otherwise prints an error and kill ros
    
    `;
  }

  static Resolve(msg) {
    // deep-construct a valid message object instance of whatever was passed in
    if (typeof msg !== 'object' || msg === null) {
      msg = {};
    }
    const resolved = new CoeffsRequest(null);
    if (msg.ready !== undefined) {
      resolved.ready = msg.ready;
    }
    else {
      resolved.ready = false
    }

    return resolved;
    }
};

class CoeffsResponse {
  constructor(initObj={}) {
    if (initObj === null) {
      // initObj === null is a special case for deserialization where we don't initialize fields
      this.coeffs = null;
    }
    else {
      if (initObj.hasOwnProperty('coeffs')) {
        this.coeffs = initObj.coeffs
      }
      else {
        this.coeffs = [];
      }
    }
  }

  static serialize(obj, buffer, bufferOffset) {
    // Serializes a message object of type CoeffsResponse
    // Serialize message field [coeffs]
    bufferOffset = _arraySerializer.float32(obj.coeffs, buffer, bufferOffset, null);
    return bufferOffset;
  }

  static deserialize(buffer, bufferOffset=[0]) {
    //deserializes a message object of type CoeffsResponse
    let len;
    let data = new CoeffsResponse(null);
    // Deserialize message field [coeffs]
    data.coeffs = _arrayDeserializer.float32(buffer, bufferOffset, null)
    return data;
  }

  static getMessageSize(object) {
    let length = 0;
    length += 4 * object.coeffs.length;
    return length + 4;
  }

  static datatype() {
    // Returns string type for a service object
    return 'tiago_iaslab_simulation/CoeffsResponse';
  }

  static md5sum() {
    //Returns md5sum for a message object
    return 'b71cfea2226a7084c29c0b3987c6e123';
  }

  static messageDefinition() {
    // Returns full string definition for message
    return `
    float32[] coeffs
    
    
    `;
  }

  static Resolve(msg) {
    // deep-construct a valid message object instance of whatever was passed in
    if (typeof msg !== 'object' || msg === null) {
      msg = {};
    }
    const resolved = new CoeffsResponse(null);
    if (msg.coeffs !== undefined) {
      resolved.coeffs = msg.coeffs;
    }
    else {
      resolved.coeffs = []
    }

    return resolved;
    }
};

module.exports = {
  Request: CoeffsRequest,
  Response: CoeffsResponse,
  md5sum() { return '970e0019261c40a0b3c0c70ce65f2917'; },
  datatype() { return 'tiago_iaslab_simulation/Coeffs'; }
};
