; Auto-generated. Do not edit!


(cl:in-package assignment2_package-msg)


;//! \htmlinclude ObjectPoseArray.msg.html

(cl:defclass <ObjectPoseArray> (roslisp-msg-protocol:ros-message)
  ((objects
    :reader objects
    :initarg :objects
    :type (cl:vector assignment2_package-msg:ObjectPose)
   :initform (cl:make-array 0 :element-type 'assignment2_package-msg:ObjectPose :initial-element (cl:make-instance 'assignment2_package-msg:ObjectPose))))
)

(cl:defclass ObjectPoseArray (<ObjectPoseArray>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <ObjectPoseArray>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'ObjectPoseArray)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name assignment2_package-msg:<ObjectPoseArray> is deprecated: use assignment2_package-msg:ObjectPoseArray instead.")))

(cl:ensure-generic-function 'objects-val :lambda-list '(m))
(cl:defmethod objects-val ((m <ObjectPoseArray>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader assignment2_package-msg:objects-val is deprecated.  Use assignment2_package-msg:objects instead.")
  (objects m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <ObjectPoseArray>) ostream)
  "Serializes a message object of type '<ObjectPoseArray>"
  (cl:let ((__ros_arr_len (cl:length (cl:slot-value msg 'objects))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) __ros_arr_len) ostream))
  (cl:map cl:nil #'(cl:lambda (ele) (roslisp-msg-protocol:serialize ele ostream))
   (cl:slot-value msg 'objects))
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <ObjectPoseArray>) istream)
  "Deserializes a message object of type '<ObjectPoseArray>"
  (cl:let ((__ros_arr_len 0))
    (cl:setf (cl:ldb (cl:byte 8 0) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 8) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 16) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 24) __ros_arr_len) (cl:read-byte istream))
  (cl:setf (cl:slot-value msg 'objects) (cl:make-array __ros_arr_len))
  (cl:let ((vals (cl:slot-value msg 'objects)))
    (cl:dotimes (i __ros_arr_len)
    (cl:setf (cl:aref vals i) (cl:make-instance 'assignment2_package-msg:ObjectPose))
  (roslisp-msg-protocol:deserialize (cl:aref vals i) istream))))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<ObjectPoseArray>)))
  "Returns string type for a message object of type '<ObjectPoseArray>"
  "assignment2_package/ObjectPoseArray")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'ObjectPoseArray)))
  "Returns string type for a message object of type 'ObjectPoseArray"
  "assignment2_package/ObjectPoseArray")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<ObjectPoseArray>)))
  "Returns md5sum for a message object of type '<ObjectPoseArray>"
  "5d14148e7c4ffcd67a5e7838038a1331")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'ObjectPoseArray)))
  "Returns md5sum for a message object of type 'ObjectPoseArray"
  "5d14148e7c4ffcd67a5e7838038a1331")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<ObjectPoseArray>)))
  "Returns full string definition for message of type '<ObjectPoseArray>"
  (cl:format cl:nil "assignment2_package/ObjectPose[] objects~%================================================================================~%MSG: assignment2_package/ObjectPose~%uint32 id~%geometry_msgs/PoseStamped pose~%~%================================================================================~%MSG: geometry_msgs/PoseStamped~%# A Pose with reference coordinate frame and timestamp~%Header header~%Pose pose~%~%================================================================================~%MSG: std_msgs/Header~%# Standard metadata for higher-level stamped data types.~%# This is generally used to communicate timestamped data ~%# in a particular coordinate frame.~%# ~%# sequence ID: consecutively increasing ID ~%uint32 seq~%#Two-integer timestamp that is expressed as:~%# * stamp.sec: seconds (stamp_secs) since epoch (in Python the variable is called 'secs')~%# * stamp.nsec: nanoseconds since stamp_secs (in Python the variable is called 'nsecs')~%# time-handling sugar is provided by the client library~%time stamp~%#Frame this data is associated with~%string frame_id~%~%================================================================================~%MSG: geometry_msgs/Pose~%# A representation of pose in free space, composed of position and orientation. ~%Point position~%Quaternion orientation~%~%================================================================================~%MSG: geometry_msgs/Point~%# This contains the position of a point in free space~%float64 x~%float64 y~%float64 z~%~%================================================================================~%MSG: geometry_msgs/Quaternion~%# This represents an orientation in free space in quaternion form.~%~%float64 x~%float64 y~%float64 z~%float64 w~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'ObjectPoseArray)))
  "Returns full string definition for message of type 'ObjectPoseArray"
  (cl:format cl:nil "assignment2_package/ObjectPose[] objects~%================================================================================~%MSG: assignment2_package/ObjectPose~%uint32 id~%geometry_msgs/PoseStamped pose~%~%================================================================================~%MSG: geometry_msgs/PoseStamped~%# A Pose with reference coordinate frame and timestamp~%Header header~%Pose pose~%~%================================================================================~%MSG: std_msgs/Header~%# Standard metadata for higher-level stamped data types.~%# This is generally used to communicate timestamped data ~%# in a particular coordinate frame.~%# ~%# sequence ID: consecutively increasing ID ~%uint32 seq~%#Two-integer timestamp that is expressed as:~%# * stamp.sec: seconds (stamp_secs) since epoch (in Python the variable is called 'secs')~%# * stamp.nsec: nanoseconds since stamp_secs (in Python the variable is called 'nsecs')~%# time-handling sugar is provided by the client library~%time stamp~%#Frame this data is associated with~%string frame_id~%~%================================================================================~%MSG: geometry_msgs/Pose~%# A representation of pose in free space, composed of position and orientation. ~%Point position~%Quaternion orientation~%~%================================================================================~%MSG: geometry_msgs/Point~%# This contains the position of a point in free space~%float64 x~%float64 y~%float64 z~%~%================================================================================~%MSG: geometry_msgs/Quaternion~%# This represents an orientation in free space in quaternion form.~%~%float64 x~%float64 y~%float64 z~%float64 w~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <ObjectPoseArray>))
  (cl:+ 0
     4 (cl:reduce #'cl:+ (cl:slot-value msg 'objects) :key #'(cl:lambda (ele) (cl:declare (cl:ignorable ele)) (cl:+ (roslisp-msg-protocol:serialization-length ele))))
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <ObjectPoseArray>))
  "Converts a ROS message object to a list"
  (cl:list 'ObjectPoseArray
    (cl:cons ':objects (objects msg))
))
