// Auto-generated. Do not edit!

// (in-package kinect_v2.msg)


"use strict";

const _serializer = _ros_msg_utils.Serialize;
const _arraySerializer = _serializer.Array;
const _deserializer = _ros_msg_utils.Deserialize;
const _arrayDeserializer = _deserializer.Array;
const _finder = _ros_msg_utils.Find;
const _getByteLength = _ros_msg_utils.getByteLength;
let geometry_msgs = _finder('geometry_msgs');

//-----------------------------------------------------------

class BodyJoints {
  constructor(initObj={}) {
    if (initObj === null) {
      // initObj === null is a special case for deserialization where we don't initialize fields
      this.user_id = null;
      this.tracked = null;
      this.joints = null;
    }
    else {
      if (initObj.hasOwnProperty('user_id')) {
        this.user_id = initObj.user_id
      }
      else {
        this.user_id = 0;
      }
      if (initObj.hasOwnProperty('tracked')) {
        this.tracked = initObj.tracked
      }
      else {
        this.tracked = '';
      }
      if (initObj.hasOwnProperty('joints')) {
        this.joints = initObj.joints
      }
      else {
        this.joints = new Array(16).fill(new geometry_msgs.msg.Pose());
      }
    }
  }

  static serialize(obj, buffer, bufferOffset) {
    // Serializes a message object of type BodyJoints
    // Serialize message field [user_id]
    bufferOffset = _serializer.int32(obj.user_id, buffer, bufferOffset);
    // Serialize message field [tracked]
    bufferOffset = _serializer.string(obj.tracked, buffer, bufferOffset);
    // Check that the constant length array field [joints] has the right length
    if (obj.joints.length !== 16) {
      throw new Error('Unable to serialize array field joints - length must be 16')
    }
    // Serialize message field [joints]
    obj.joints.forEach((val) => {
      bufferOffset = geometry_msgs.msg.Pose.serialize(val, buffer, bufferOffset);
    });
    return bufferOffset;
  }

  static deserialize(buffer, bufferOffset=[0]) {
    //deserializes a message object of type BodyJoints
    let len;
    let data = new BodyJoints(null);
    // Deserialize message field [user_id]
    data.user_id = _deserializer.int32(buffer, bufferOffset);
    // Deserialize message field [tracked]
    data.tracked = _deserializer.string(buffer, bufferOffset);
    // Deserialize message field [joints]
    len = 16;
    data.joints = new Array(len);
    for (let i = 0; i < len; ++i) {
      data.joints[i] = geometry_msgs.msg.Pose.deserialize(buffer, bufferOffset)
    }
    return data;
  }

  static getMessageSize(object) {
    let length = 0;
    length += object.tracked.length;
    return length + 904;
  }

  static datatype() {
    // Returns string type for a message object
    return 'kinect_v2/BodyJoints';
  }

  static md5sum() {
    //Returns md5sum for a message object
    return '61535990ee807ee844649627b51297c2';
  }

  static messageDefinition() {
    // Returns full string definition for message
    return `
    int32 user_id
    string tracked
    geometry_msgs/Pose[16] joints
    
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
    const resolved = new BodyJoints(null);
    if (msg.user_id !== undefined) {
      resolved.user_id = msg.user_id;
    }
    else {
      resolved.user_id = 0
    }

    if (msg.tracked !== undefined) {
      resolved.tracked = msg.tracked;
    }
    else {
      resolved.tracked = ''
    }

    if (msg.joints !== undefined) {
      resolved.joints = new Array(16)
      for (let i = 0; i < resolved.joints.length; ++i) {
        if (msg.joints.length > i) {
          resolved.joints[i] = geometry_msgs.msg.Pose.Resolve(msg.joints[i]);
        }
        else {
          resolved.joints[i] = new geometry_msgs.msg.Pose();
        }
      }
    }
    else {
      resolved.joints = new Array(16).fill(new geometry_msgs.msg.Pose())
    }

    return resolved;
    }
};

module.exports = BodyJoints;
