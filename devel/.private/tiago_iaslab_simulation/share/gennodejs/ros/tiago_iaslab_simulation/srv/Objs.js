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

class ObjsRequest {
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
    // Serializes a message object of type ObjsRequest
    // Serialize message field [ready]
    bufferOffset = _serializer.bool(obj.ready, buffer, bufferOffset);
    return bufferOffset;
  }

  static deserialize(buffer, bufferOffset=[0]) {
    //deserializes a message object of type ObjsRequest
    let len;
    let data = new ObjsRequest(null);
    // Deserialize message field [ready]
    data.ready = _deserializer.bool(buffer, bufferOffset);
    return data;
  }

  static getMessageSize(object) {
    return 1;
  }

  static datatype() {
    // Returns string type for a service object
    return 'tiago_iaslab_simulation/ObjsRequest';
  }

  static md5sum() {
    //Returns md5sum for a message object
    return '6f378c6311f9e6ccd2cd8c5b327003f1';
  }

  static messageDefinition() {
    // Returns full string definition for message
    return `
    bool ready          #if true apriltag_ids sends the apriltags IDs, otherwise prints an error and kill ros
    
    `;
  }

  static Resolve(msg) {
    // deep-construct a valid message object instance of whatever was passed in
    if (typeof msg !== 'object' || msg === null) {
      msg = {};
    }
    const resolved = new ObjsRequest(null);
    if (msg.ready !== undefined) {
      resolved.ready = msg.ready;
    }
    else {
      resolved.ready = false
    }

    return resolved;
    }
};

class ObjsResponse {
  constructor(initObj={}) {
    if (initObj === null) {
      // initObj === null is a special case for deserialization where we don't initialize fields
      this.ids = null;
    }
    else {
      if (initObj.hasOwnProperty('ids')) {
        this.ids = initObj.ids
      }
      else {
        this.ids = [];
      }
    }
  }

  static serialize(obj, buffer, bufferOffset) {
    // Serializes a message object of type ObjsResponse
    // Serialize message field [ids]
    bufferOffset = _arraySerializer.int32(obj.ids, buffer, bufferOffset, null);
    return bufferOffset;
  }

  static deserialize(buffer, bufferOffset=[0]) {
    //deserializes a message object of type ObjsResponse
    let len;
    let data = new ObjsResponse(null);
    // Deserialize message field [ids]
    data.ids = _arrayDeserializer.int32(buffer, bufferOffset, null)
    return data;
  }

  static getMessageSize(object) {
    let length = 0;
    length += 4 * object.ids.length;
    return length + 4;
  }

  static datatype() {
    // Returns string type for a service object
    return 'tiago_iaslab_simulation/ObjsResponse';
  }

  static md5sum() {
    //Returns md5sum for a message object
    return '4f22efebf407aadba2ecc69df353d113';
  }

  static messageDefinition() {
    // Returns full string definition for message
    return `
    int32[] ids
    
    
    `;
  }

  static Resolve(msg) {
    // deep-construct a valid message object instance of whatever was passed in
    if (typeof msg !== 'object' || msg === null) {
      msg = {};
    }
    const resolved = new ObjsResponse(null);
    if (msg.ids !== undefined) {
      resolved.ids = msg.ids;
    }
    else {
      resolved.ids = []
    }

    return resolved;
    }
};

module.exports = {
  Request: ObjsRequest,
  Response: ObjsResponse,
  md5sum() { return 'b0d667d823cb9909fa2a290df26f97da'; },
  datatype() { return 'tiago_iaslab_simulation/Objs'; }
};
