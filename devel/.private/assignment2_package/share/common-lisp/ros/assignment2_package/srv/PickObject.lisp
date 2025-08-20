; Auto-generated. Do not edit!


(cl:in-package assignment2_package-srv)


;//! \htmlinclude PickObject-request.msg.html

(cl:defclass <PickObject-request> (roslisp-msg-protocol:ros-message)
  ((target
    :reader target
    :initarg :target
    :type geometry_msgs-msg:PoseStamped
    :initform (cl:make-instance 'geometry_msgs-msg:PoseStamped))
   (target_id
    :reader target_id
    :initarg :target_id
    :type cl:integer
    :initform 0))
)

(cl:defclass PickObject-request (<PickObject-request>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <PickObject-request>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'PickObject-request)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name assignment2_package-srv:<PickObject-request> is deprecated: use assignment2_package-srv:PickObject-request instead.")))

(cl:ensure-generic-function 'target-val :lambda-list '(m))
(cl:defmethod target-val ((m <PickObject-request>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader assignment2_package-srv:target-val is deprecated.  Use assignment2_package-srv:target instead.")
  (target m))

(cl:ensure-generic-function 'target_id-val :lambda-list '(m))
(cl:defmethod target_id-val ((m <PickObject-request>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader assignment2_package-srv:target_id-val is deprecated.  Use assignment2_package-srv:target_id instead.")
  (target_id m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <PickObject-request>) ostream)
  "Serializes a message object of type '<PickObject-request>"
  (roslisp-msg-protocol:serialize (cl:slot-value msg 'target) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'target_id)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 8) (cl:slot-value msg 'target_id)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 16) (cl:slot-value msg 'target_id)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 24) (cl:slot-value msg 'target_id)) ostream)
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <PickObject-request>) istream)
  "Deserializes a message object of type '<PickObject-request>"
  (roslisp-msg-protocol:deserialize (cl:slot-value msg 'target) istream)
    (cl:setf (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'target_id)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 8) (cl:slot-value msg 'target_id)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 16) (cl:slot-value msg 'target_id)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 24) (cl:slot-value msg 'target_id)) (cl:read-byte istream))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<PickObject-request>)))
  "Returns string type for a service object of type '<PickObject-request>"
  "assignment2_package/PickObjectRequest")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'PickObject-request)))
  "Returns string type for a service object of type 'PickObject-request"
  "assignment2_package/PickObjectRequest")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<PickObject-request>)))
  "Returns md5sum for a message object of type '<PickObject-request>"
  "1353e22d17c63a0b3e01bd479f8a9f82")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'PickObject-request)))
  "Returns md5sum for a message object of type 'PickObject-request"
  "1353e22d17c63a0b3e01bd479f8a9f82")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<PickObject-request>)))
  "Returns full string definition for message of type '<PickObject-request>"
  (cl:format cl:nil "geometry_msgs/PoseStamped target~%uint32 target_id~%~%================================================================================~%MSG: geometry_msgs/PoseStamped~%# A Pose with reference coordinate frame and timestamp~%Header header~%Pose pose~%~%================================================================================~%MSG: std_msgs/Header~%# Standard metadata for higher-level stamped data types.~%# This is generally used to communicate timestamped data ~%# in a particular coordinate frame.~%# ~%# sequence ID: consecutively increasing ID ~%uint32 seq~%#Two-integer timestamp that is expressed as:~%# * stamp.sec: seconds (stamp_secs) since epoch (in Python the variable is called 'secs')~%# * stamp.nsec: nanoseconds since stamp_secs (in Python the variable is called 'nsecs')~%# time-handling sugar is provided by the client library~%time stamp~%#Frame this data is associated with~%string frame_id~%~%================================================================================~%MSG: geometry_msgs/Pose~%# A representation of pose in free space, composed of position and orientation. ~%Point position~%Quaternion orientation~%~%================================================================================~%MSG: geometry_msgs/Point~%# This contains the position of a point in free space~%float64 x~%float64 y~%float64 z~%~%================================================================================~%MSG: geometry_msgs/Quaternion~%# This represents an orientation in free space in quaternion form.~%~%float64 x~%float64 y~%float64 z~%float64 w~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'PickObject-request)))
  "Returns full string definition for message of type 'PickObject-request"
  (cl:format cl:nil "geometry_msgs/PoseStamped target~%uint32 target_id~%~%================================================================================~%MSG: geometry_msgs/PoseStamped~%# A Pose with reference coordinate frame and timestamp~%Header header~%Pose pose~%~%================================================================================~%MSG: std_msgs/Header~%# Standard metadata for higher-level stamped data types.~%# This is generally used to communicate timestamped data ~%# in a particular coordinate frame.~%# ~%# sequence ID: consecutively increasing ID ~%uint32 seq~%#Two-integer timestamp that is expressed as:~%# * stamp.sec: seconds (stamp_secs) since epoch (in Python the variable is called 'secs')~%# * stamp.nsec: nanoseconds since stamp_secs (in Python the variable is called 'nsecs')~%# time-handling sugar is provided by the client library~%time stamp~%#Frame this data is associated with~%string frame_id~%~%================================================================================~%MSG: geometry_msgs/Pose~%# A representation of pose in free space, composed of position and orientation. ~%Point position~%Quaternion orientation~%~%================================================================================~%MSG: geometry_msgs/Point~%# This contains the position of a point in free space~%float64 x~%float64 y~%float64 z~%~%================================================================================~%MSG: geometry_msgs/Quaternion~%# This represents an orientation in free space in quaternion form.~%~%float64 x~%float64 y~%float64 z~%float64 w~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <PickObject-request>))
  (cl:+ 0
     (roslisp-msg-protocol:serialization-length (cl:slot-value msg 'target))
     4
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <PickObject-request>))
  "Converts a ROS message object to a list"
  (cl:list 'PickObject-request
    (cl:cons ':target (target msg))
    (cl:cons ':target_id (target_id msg))
))
;//! \htmlinclude PickObject-response.msg.html

(cl:defclass <PickObject-response> (roslisp-msg-protocol:ros-message)
  ((success
    :reader success
    :initarg :success
    :type cl:boolean
    :initform cl:nil))
)

(cl:defclass PickObject-response (<PickObject-response>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <PickObject-response>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'PickObject-response)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name assignment2_package-srv:<PickObject-response> is deprecated: use assignment2_package-srv:PickObject-response instead.")))

(cl:ensure-generic-function 'success-val :lambda-list '(m))
(cl:defmethod success-val ((m <PickObject-response>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader assignment2_package-srv:success-val is deprecated.  Use assignment2_package-srv:success instead.")
  (success m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <PickObject-response>) ostream)
  "Serializes a message object of type '<PickObject-response>"
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:if (cl:slot-value msg 'success) 1 0)) ostream)
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <PickObject-response>) istream)
  "Deserializes a message object of type '<PickObject-response>"
    (cl:setf (cl:slot-value msg 'success) (cl:not (cl:zerop (cl:read-byte istream))))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<PickObject-response>)))
  "Returns string type for a service object of type '<PickObject-response>"
  "assignment2_package/PickObjectResponse")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'PickObject-response)))
  "Returns string type for a service object of type 'PickObject-response"
  "assignment2_package/PickObjectResponse")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<PickObject-response>)))
  "Returns md5sum for a message object of type '<PickObject-response>"
  "1353e22d17c63a0b3e01bd479f8a9f82")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'PickObject-response)))
  "Returns md5sum for a message object of type 'PickObject-response"
  "1353e22d17c63a0b3e01bd479f8a9f82")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<PickObject-response>)))
  "Returns full string definition for message of type '<PickObject-response>"
  (cl:format cl:nil "bool success~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'PickObject-response)))
  "Returns full string definition for message of type 'PickObject-response"
  (cl:format cl:nil "bool success~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <PickObject-response>))
  (cl:+ 0
     1
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <PickObject-response>))
  "Converts a ROS message object to a list"
  (cl:list 'PickObject-response
    (cl:cons ':success (success msg))
))
(cl:defmethod roslisp-msg-protocol:service-request-type ((msg (cl:eql 'PickObject)))
  'PickObject-request)
(cl:defmethod roslisp-msg-protocol:service-response-type ((msg (cl:eql 'PickObject)))
  'PickObject-response)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'PickObject)))
  "Returns string type for a service object of type '<PickObject>"
  "assignment2_package/PickObject")