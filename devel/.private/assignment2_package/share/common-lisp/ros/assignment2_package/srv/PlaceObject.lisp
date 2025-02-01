; Auto-generated. Do not edit!


(cl:in-package assignment2_package-srv)


;//! \htmlinclude PlaceObject-request.msg.html

(cl:defclass <PlaceObject-request> (roslisp-msg-protocol:ros-message)
  ((target_pose
    :reader target_pose
    :initarg :target_pose
    :type geometry_msgs-msg:PoseStamped
    :initform (cl:make-instance 'geometry_msgs-msg:PoseStamped)))
)

(cl:defclass PlaceObject-request (<PlaceObject-request>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <PlaceObject-request>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'PlaceObject-request)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name assignment2_package-srv:<PlaceObject-request> is deprecated: use assignment2_package-srv:PlaceObject-request instead.")))

(cl:ensure-generic-function 'target_pose-val :lambda-list '(m))
(cl:defmethod target_pose-val ((m <PlaceObject-request>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader assignment2_package-srv:target_pose-val is deprecated.  Use assignment2_package-srv:target_pose instead.")
  (target_pose m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <PlaceObject-request>) ostream)
  "Serializes a message object of type '<PlaceObject-request>"
  (roslisp-msg-protocol:serialize (cl:slot-value msg 'target_pose) ostream)
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <PlaceObject-request>) istream)
  "Deserializes a message object of type '<PlaceObject-request>"
  (roslisp-msg-protocol:deserialize (cl:slot-value msg 'target_pose) istream)
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<PlaceObject-request>)))
  "Returns string type for a service object of type '<PlaceObject-request>"
  "assignment2_package/PlaceObjectRequest")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'PlaceObject-request)))
  "Returns string type for a service object of type 'PlaceObject-request"
  "assignment2_package/PlaceObjectRequest")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<PlaceObject-request>)))
  "Returns md5sum for a message object of type '<PlaceObject-request>"
  "9a1b28e959609263fbb3c7ffb98e17db")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'PlaceObject-request)))
  "Returns md5sum for a message object of type 'PlaceObject-request"
  "9a1b28e959609263fbb3c7ffb98e17db")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<PlaceObject-request>)))
  "Returns full string definition for message of type '<PlaceObject-request>"
  (cl:format cl:nil "# PlaceObject.srv~%geometry_msgs/PoseStamped target_pose~%~%================================================================================~%MSG: geometry_msgs/PoseStamped~%# A Pose with reference coordinate frame and timestamp~%Header header~%Pose pose~%~%================================================================================~%MSG: std_msgs/Header~%# Standard metadata for higher-level stamped data types.~%# This is generally used to communicate timestamped data ~%# in a particular coordinate frame.~%# ~%# sequence ID: consecutively increasing ID ~%uint32 seq~%#Two-integer timestamp that is expressed as:~%# * stamp.sec: seconds (stamp_secs) since epoch (in Python the variable is called 'secs')~%# * stamp.nsec: nanoseconds since stamp_secs (in Python the variable is called 'nsecs')~%# time-handling sugar is provided by the client library~%time stamp~%#Frame this data is associated with~%string frame_id~%~%================================================================================~%MSG: geometry_msgs/Pose~%# A representation of pose in free space, composed of position and orientation. ~%Point position~%Quaternion orientation~%~%================================================================================~%MSG: geometry_msgs/Point~%# This contains the position of a point in free space~%float64 x~%float64 y~%float64 z~%~%================================================================================~%MSG: geometry_msgs/Quaternion~%# This represents an orientation in free space in quaternion form.~%~%float64 x~%float64 y~%float64 z~%float64 w~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'PlaceObject-request)))
  "Returns full string definition for message of type 'PlaceObject-request"
  (cl:format cl:nil "# PlaceObject.srv~%geometry_msgs/PoseStamped target_pose~%~%================================================================================~%MSG: geometry_msgs/PoseStamped~%# A Pose with reference coordinate frame and timestamp~%Header header~%Pose pose~%~%================================================================================~%MSG: std_msgs/Header~%# Standard metadata for higher-level stamped data types.~%# This is generally used to communicate timestamped data ~%# in a particular coordinate frame.~%# ~%# sequence ID: consecutively increasing ID ~%uint32 seq~%#Two-integer timestamp that is expressed as:~%# * stamp.sec: seconds (stamp_secs) since epoch (in Python the variable is called 'secs')~%# * stamp.nsec: nanoseconds since stamp_secs (in Python the variable is called 'nsecs')~%# time-handling sugar is provided by the client library~%time stamp~%#Frame this data is associated with~%string frame_id~%~%================================================================================~%MSG: geometry_msgs/Pose~%# A representation of pose in free space, composed of position and orientation. ~%Point position~%Quaternion orientation~%~%================================================================================~%MSG: geometry_msgs/Point~%# This contains the position of a point in free space~%float64 x~%float64 y~%float64 z~%~%================================================================================~%MSG: geometry_msgs/Quaternion~%# This represents an orientation in free space in quaternion form.~%~%float64 x~%float64 y~%float64 z~%float64 w~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <PlaceObject-request>))
  (cl:+ 0
     (roslisp-msg-protocol:serialization-length (cl:slot-value msg 'target_pose))
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <PlaceObject-request>))
  "Converts a ROS message object to a list"
  (cl:list 'PlaceObject-request
    (cl:cons ':target_pose (target_pose msg))
))
;//! \htmlinclude PlaceObject-response.msg.html

(cl:defclass <PlaceObject-response> (roslisp-msg-protocol:ros-message)
  ((success
    :reader success
    :initarg :success
    :type cl:boolean
    :initform cl:nil)
   (message
    :reader message
    :initarg :message
    :type cl:string
    :initform ""))
)

(cl:defclass PlaceObject-response (<PlaceObject-response>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <PlaceObject-response>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'PlaceObject-response)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name assignment2_package-srv:<PlaceObject-response> is deprecated: use assignment2_package-srv:PlaceObject-response instead.")))

(cl:ensure-generic-function 'success-val :lambda-list '(m))
(cl:defmethod success-val ((m <PlaceObject-response>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader assignment2_package-srv:success-val is deprecated.  Use assignment2_package-srv:success instead.")
  (success m))

(cl:ensure-generic-function 'message-val :lambda-list '(m))
(cl:defmethod message-val ((m <PlaceObject-response>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader assignment2_package-srv:message-val is deprecated.  Use assignment2_package-srv:message instead.")
  (message m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <PlaceObject-response>) ostream)
  "Serializes a message object of type '<PlaceObject-response>"
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:if (cl:slot-value msg 'success) 1 0)) ostream)
  (cl:let ((__ros_str_len (cl:length (cl:slot-value msg 'message))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) __ros_str_len) ostream))
  (cl:map cl:nil #'(cl:lambda (c) (cl:write-byte (cl:char-code c) ostream)) (cl:slot-value msg 'message))
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <PlaceObject-response>) istream)
  "Deserializes a message object of type '<PlaceObject-response>"
    (cl:setf (cl:slot-value msg 'success) (cl:not (cl:zerop (cl:read-byte istream))))
    (cl:let ((__ros_str_len 0))
      (cl:setf (cl:ldb (cl:byte 8 0) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:slot-value msg 'message) (cl:make-string __ros_str_len))
      (cl:dotimes (__ros_str_idx __ros_str_len msg)
        (cl:setf (cl:char (cl:slot-value msg 'message) __ros_str_idx) (cl:code-char (cl:read-byte istream)))))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<PlaceObject-response>)))
  "Returns string type for a service object of type '<PlaceObject-response>"
  "assignment2_package/PlaceObjectResponse")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'PlaceObject-response)))
  "Returns string type for a service object of type 'PlaceObject-response"
  "assignment2_package/PlaceObjectResponse")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<PlaceObject-response>)))
  "Returns md5sum for a message object of type '<PlaceObject-response>"
  "9a1b28e959609263fbb3c7ffb98e17db")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'PlaceObject-response)))
  "Returns md5sum for a message object of type 'PlaceObject-response"
  "9a1b28e959609263fbb3c7ffb98e17db")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<PlaceObject-response>)))
  "Returns full string definition for message of type '<PlaceObject-response>"
  (cl:format cl:nil "bool success~%string message~%~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'PlaceObject-response)))
  "Returns full string definition for message of type 'PlaceObject-response"
  (cl:format cl:nil "bool success~%string message~%~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <PlaceObject-response>))
  (cl:+ 0
     1
     4 (cl:length (cl:slot-value msg 'message))
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <PlaceObject-response>))
  "Converts a ROS message object to a list"
  (cl:list 'PlaceObject-response
    (cl:cons ':success (success msg))
    (cl:cons ':message (message msg))
))
(cl:defmethod roslisp-msg-protocol:service-request-type ((msg (cl:eql 'PlaceObject)))
  'PlaceObject-request)
(cl:defmethod roslisp-msg-protocol:service-response-type ((msg (cl:eql 'PlaceObject)))
  'PlaceObject-response)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'PlaceObject)))
  "Returns string type for a service object of type '<PlaceObject>"
  "assignment2_package/PlaceObject")