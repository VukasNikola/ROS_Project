// Auto-generated. Do not edit!

// (in-package assignment2_package.srv)


"use strict";

const _serializer = _ros_msg_utils.Serialize;
const _arraySerializer = _serializer.Array;
const _deserializer = _ros_msg_utils.Deserialize;
const _arrayDeserializer = _deserializer.Array;
const _finder = _ros_msg_utils.Find;
const _getByteLength = _ros_msg_utils.getByteLength;

//-----------------------------------------------------------

let geometry_msgs = _finder('geometry_msgs');

//-----------------------------------------------------------

class GetObjectPoseRequest {
  constructor(initObj={}) {
    if (initObj === null) {
      // initObj === null is a special case for deserialization where we don't initialize fields
    }
    else {
    }
  }

  static serialize(obj, buffer, bufferOffset) {
    // Serializes a message object of type GetObjectPoseRequest
    return bufferOffset;
  }

  static deserialize(buffer, bufferOffset=[0]) {
    //deserializes a message object of type GetObjectPoseRequest
    let len;
    let data = new GetObjectPoseRequest(null);
    return data;
  }

  static getMessageSize(object) {
    return 0;
  }

  static datatype() {
    // Returns string type for a service object
    return 'assignment2_package/GetObjectPoseRequest';
  }

  static md5sum() {
    //Returns md5sum for a message object
    return 'd41d8cd98f00b204e9800998ecf8427e';
  }

  static messageDefinition() {
    // Returns full string definition for message
    return `
    #No request
    
    `;
  }

  static Resolve(msg) {
    // deep-construct a valid message object instance of whatever was passed in
    if (typeof msg !== 'object' || msg === null) {
      msg = {};
    }
    const resolved = new GetObjectPoseRequest(null);
    return resolved;
    }
};

class GetObjectPoseResponse {
  constructor(initObj={}) {
    if (initObj === null) {
      // initObj === null is a special case for deserialization where we don't initialize fields
      this.obj_id = null;
      this.obj_pose = null;
    }
    else {
      if (initObj.hasOwnProperty('obj_id')) {
        this.obj_id = initObj.obj_id
      }
      else {
        this.obj_id = 0;
      }
      if (initObj.hasOwnProperty('obj_pose')) {
        this.obj_pose = initObj.obj_pose
      }
      else {
        this.obj_pose = new geometry_msgs.msg.PoseStamped();
      }
    }
  }

  static serialize(obj, buffer, bufferOffset) {
    // Serializes a message object of type GetObjectPoseResponse
    // Serialize message field [obj_id]
    bufferOffset = _serializer.uint32(obj.obj_id, buffer, bufferOffset);
    // Serialize message field [obj_pose]
    bufferOffset = geometry_msgs.msg.PoseStamped.serialize(obj.obj_pose, buffer, bufferOffset);
    return bufferOffset;
  }

  static deserialize(buffer, bufferOffset=[0]) {
    //deserializes a message object of type GetObjectPoseResponse
    let len;
    let data = new GetObjectPoseResponse(null);
    // Deserialize message field [obj_id]
    data.obj_id = _deserializer.uint32(buffer, bufferOffset);
    // Deserialize message field [obj_pose]
    data.obj_pose = geometry_msgs.msg.PoseStamped.deserialize(buffer, bufferOffset);
    return data;
  }

  static getMessageSize(object) {
    let length = 0;
    length += geometry_msgs.msg.PoseStamped.getMessageSize(object.obj_pose);
    return length + 4;
  }

  static datatype() {
    // Returns string type for a service object
    return 'assignment2_package/GetObjectPoseResponse';
  }

  static md5sum() {
    //Returns md5sum for a message object
    return 'd6d4f3bebbf31073267f6fff2a596a9a';
  }

  static messageDefinition() {
    // Returns full string definition for message
    return `
    uint32 obj_id
    geometry_msgs/PoseStamped obj_pose
    
    ================================================================================
    MSG: geometry_msgs/PoseStamped
    # A Pose with reference coordinate frame and timestamp
    Header header
    Pose pose
    
    ================================================================================
    MSG: std_msgs/Header
    # Standard metadata for higher-level stamped data types.
    # This is generally used to communicate timestamped data 
    # in a particular coordinate frame.
    # 
    # sequence ID: consecutively increasing ID 
    uint32 seq
    #Two-integer timestamp that is expressed as:
    # * stamp.sec: seconds (stamp_secs) since epoch (in Python the variable is called 'secs')
    # * stamp.nsec: nanoseconds since stamp_secs (in Python the variable is called 'nsecs')
    # time-handling sugar is provided by the client library
    time stamp
    #Frame this data is associated with
    string frame_id
    
    ================================================================================
    MSG: geometry_msgs/Pose
    # A representation of pose in free space, composed of position and orientation. 
    Point position
    Quaternion orientation
    
    ================================================================================
    MSG: geometry_msgs/Point
    # This contains the position of a point in free space
    float64 x
    float64 y
    float64 z
    
    ================================================================================
    MSG: geometry_msgs/Quaternion
    # This represents an orientation in free space in quaternion form.
    
    float64 x
    float64 y
    float64 z
    float64 w
    
    `;
  }

  static Resolve(msg) {
    // deep-construct a valid message object instance of whatever was passed in
    if (typeof msg !== 'object' || msg === null) {
      msg = {};
    }
    const resolved = new GetObjectPoseResponse(null);
    if (msg.obj_id !== undefined) {
      resolved.obj_id = msg.obj_id;
    }
    else {
      resolved.obj_id = 0
    }

    if (msg.obj_pose !== undefined) {
      resolved.obj_pose = geometry_msgs.msg.PoseStamped.Resolve(msg.obj_pose)
    }
    else {
      resolved.obj_pose = new geometry_msgs.msg.PoseStamped()
    }

    return resolved;
    }
};

module.exports = {
  Request: GetObjectPoseRequest,
  Response: GetObjectPoseResponse,
  md5sum() { return 'd6d4f3bebbf31073267f6fff2a596a9a'; },
  datatype() { return 'assignment2_package/GetObjectPose'; }
};
