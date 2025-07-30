; Auto-generated. Do not edit!


(cl:in-package assignment2_package-srv)


;//! \htmlinclude GetObjectPose-request.msg.html

(cl:defclass <GetObjectPose-request> (roslisp-msg-protocol:ros-message)
  ()
)

(cl:defclass GetObjectPose-request (<GetObjectPose-request>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <GetObjectPose-request>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'GetObjectPose-request)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name assignment2_package-srv:<GetObjectPose-request> is deprecated: use assignment2_package-srv:GetObjectPose-request instead.")))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <GetObjectPose-request>) ostream)
  "Serializes a message object of type '<GetObjectPose-request>"
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <GetObjectPose-request>) istream)
  "Deserializes a message object of type '<GetObjectPose-request>"
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<GetObjectPose-request>)))
  "Returns string type for a service object of type '<GetObjectPose-request>"
  "assignment2_package/GetObjectPoseRequest")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'GetObjectPose-request)))
  "Returns string type for a service object of type 'GetObjectPose-request"
  "assignment2_package/GetObjectPoseRequest")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<GetObjectPose-request>)))
  "Returns md5sum for a message object of type '<GetObjectPose-request>"
  "5174181ef2b390a67f5f6f66dd09d3ff")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'GetObjectPose-request)))
  "Returns md5sum for a message object of type 'GetObjectPose-request"
  "5174181ef2b390a67f5f6f66dd09d3ff")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<GetObjectPose-request>)))
  "Returns full string definition for message of type '<GetObjectPose-request>"
  (cl:format cl:nil "#No request~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'GetObjectPose-request)))
  "Returns full string definition for message of type 'GetObjectPose-request"
  (cl:format cl:nil "#No request~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <GetObjectPose-request>))
  (cl:+ 0
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <GetObjectPose-request>))
  "Converts a ROS message object to a list"
  (cl:list 'GetObjectPose-request
))
;//! \htmlinclude GetObjectPose-response.msg.html

(cl:defclass <GetObjectPose-response> (roslisp-msg-protocol:ros-message)
  ((obj_pose
    :reader obj_pose
    :initarg :obj_pose
    :type geometry_msgs-msg:PoseStamped
    :initform (cl:make-instance 'geometry_msgs-msg:PoseStamped)))
)

(cl:defclass GetObjectPose-response (<GetObjectPose-response>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <GetObjectPose-response>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'GetObjectPose-response)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name assignment2_package-srv:<GetObjectPose-response> is deprecated: use assignment2_package-srv:GetObjectPose-response instead.")))

(cl:ensure-generic-function 'obj_pose-val :lambda-list '(m))
(cl:defmethod obj_pose-val ((m <GetObjectPose-response>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader assignment2_package-srv:obj_pose-val is deprecated.  Use assignment2_package-srv:obj_pose instead.")
  (obj_pose m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <GetObjectPose-response>) ostream)
  "Serializes a message object of type '<GetObjectPose-response>"
  (roslisp-msg-protocol:serialize (cl:slot-value msg 'obj_pose) ostream)
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <GetObjectPose-response>) istream)
  "Deserializes a message object of type '<GetObjectPose-response>"
  (roslisp-msg-protocol:deserialize (cl:slot-value msg 'obj_pose) istream)
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<GetObjectPose-response>)))
  "Returns string type for a service object of type '<GetObjectPose-response>"
  "assignment2_package/GetObjectPoseResponse")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'GetObjectPose-response)))
  "Returns string type for a service object of type 'GetObjectPose-response"
  "assignment2_package/GetObjectPoseResponse")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<GetObjectPose-response>)))
  "Returns md5sum for a message object of type '<GetObjectPose-response>"
  "5174181ef2b390a67f5f6f66dd09d3ff")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'GetObjectPose-response)))
  "Returns md5sum for a message object of type 'GetObjectPose-response"
  "5174181ef2b390a67f5f6f66dd09d3ff")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<GetObjectPose-response>)))
  "Returns full string definition for message of type '<GetObjectPose-response>"
  (cl:format cl:nil "geometry_msgs/PoseStamped obj_pose~%~%================================================================================~%MSG: geometry_msgs/PoseStamped~%# A Pose with reference coordinate frame and timestamp~%Header header~%Pose pose~%~%================================================================================~%MSG: std_msgs/Header~%# Standard metadata for higher-level stamped data types.~%# This is generally used to communicate timestamped data ~%# in a particular coordinate frame.~%# ~%# sequence ID: consecutively increasing ID ~%uint32 seq~%#Two-integer timestamp that is expressed as:~%# * stamp.sec: seconds (stamp_secs) since epoch (in Python the variable is called 'secs')~%# * stamp.nsec: nanoseconds since stamp_secs (in Python the variable is called 'nsecs')~%# time-handling sugar is provided by the client library~%time stamp~%#Frame this data is associated with~%string frame_id~%~%================================================================================~%MSG: geometry_msgs/Pose~%# A representation of pose in free space, composed of position and orientation. ~%Point position~%Quaternion orientation~%~%================================================================================~%MSG: geometry_msgs/Point~%# This contains the position of a point in free space~%float64 x~%float64 y~%float64 z~%~%================================================================================~%MSG: geometry_msgs/Quaternion~%# This represents an orientation in free space in quaternion form.~%~%float64 x~%float64 y~%float64 z~%float64 w~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'GetObjectPose-response)))
  "Returns full string definition for message of type 'GetObjectPose-response"
  (cl:format cl:nil "geometry_msgs/PoseStamped obj_pose~%~%================================================================================~%MSG: geometry_msgs/PoseStamped~%# A Pose with reference coordinate frame and timestamp~%Header header~%Pose pose~%~%================================================================================~%MSG: std_msgs/Header~%# Standard metadata for higher-level stamped data types.~%# This is generally used to communicate timestamped data ~%# in a particular coordinate frame.~%# ~%# sequence ID: consecutively increasing ID ~%uint32 seq~%#Two-integer timestamp that is expressed as:~%# * stamp.sec: seconds (stamp_secs) since epoch (in Python the variable is called 'secs')~%# * stamp.nsec: nanoseconds since stamp_secs (in Python the variable is called 'nsecs')~%# time-handling sugar is provided by the client library~%time stamp~%#Frame this data is associated with~%string frame_id~%~%================================================================================~%MSG: geometry_msgs/Pose~%# A representation of pose in free space, composed of position and orientation. ~%Point position~%Quaternion orientation~%~%================================================================================~%MSG: geometry_msgs/Point~%# This contains the position of a point in free space~%float64 x~%float64 y~%float64 z~%~%================================================================================~%MSG: geometry_msgs/Quaternion~%# This represents an orientation in free space in quaternion form.~%~%float64 x~%float64 y~%float64 z~%float64 w~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <GetObjectPose-response>))
  (cl:+ 0
     (roslisp-msg-protocol:serialization-length (cl:slot-value msg 'obj_pose))
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <GetObjectPose-response>))
  "Converts a ROS message object to a list"
  (cl:list 'GetObjectPose-response
    (cl:cons ':obj_pose (obj_pose msg))
))
(cl:defmethod roslisp-msg-protocol:service-request-type ((msg (cl:eql 'GetObjectPose)))
  'GetObjectPose-request)
(cl:defmethod roslisp-msg-protocol:service-response-type ((msg (cl:eql 'GetObjectPose)))
  'GetObjectPose-response)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'GetObjectPose)))
  "Returns string type for a service object of type '<GetObjectPose>"
  "assignment2_package/GetObjectPose")