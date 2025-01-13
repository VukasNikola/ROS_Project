; Auto-generated. Do not edit!


(cl:in-package apriltag_finder-srv)


;//! \htmlinclude ApriltagsIDsSrv-request.msg.html

(cl:defclass <ApriltagsIDsSrv-request> (roslisp-msg-protocol:ros-message)
  ()
)

(cl:defclass ApriltagsIDsSrv-request (<ApriltagsIDsSrv-request>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <ApriltagsIDsSrv-request>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'ApriltagsIDsSrv-request)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name apriltag_finder-srv:<ApriltagsIDsSrv-request> is deprecated: use apriltag_finder-srv:ApriltagsIDsSrv-request instead.")))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <ApriltagsIDsSrv-request>) ostream)
  "Serializes a message object of type '<ApriltagsIDsSrv-request>"
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <ApriltagsIDsSrv-request>) istream)
  "Deserializes a message object of type '<ApriltagsIDsSrv-request>"
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<ApriltagsIDsSrv-request>)))
  "Returns string type for a service object of type '<ApriltagsIDsSrv-request>"
  "apriltag_finder/ApriltagsIDsSrvRequest")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'ApriltagsIDsSrv-request)))
  "Returns string type for a service object of type 'ApriltagsIDsSrv-request"
  "apriltag_finder/ApriltagsIDsSrvRequest")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<ApriltagsIDsSrv-request>)))
  "Returns md5sum for a message object of type '<ApriltagsIDsSrv-request>"
  "4f22efebf407aadba2ecc69df353d113")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'ApriltagsIDsSrv-request)))
  "Returns md5sum for a message object of type 'ApriltagsIDsSrv-request"
  "4f22efebf407aadba2ecc69df353d113")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<ApriltagsIDsSrv-request>)))
  "Returns full string definition for message of type '<ApriltagsIDsSrv-request>"
  (cl:format cl:nil "# Request~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'ApriltagsIDsSrv-request)))
  "Returns full string definition for message of type 'ApriltagsIDsSrv-request"
  (cl:format cl:nil "# Request~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <ApriltagsIDsSrv-request>))
  (cl:+ 0
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <ApriltagsIDsSrv-request>))
  "Converts a ROS message object to a list"
  (cl:list 'ApriltagsIDsSrv-request
))
;//! \htmlinclude ApriltagsIDsSrv-response.msg.html

(cl:defclass <ApriltagsIDsSrv-response> (roslisp-msg-protocol:ros-message)
  ((ids
    :reader ids
    :initarg :ids
    :type (cl:vector cl:integer)
   :initform (cl:make-array 0 :element-type 'cl:integer :initial-element 0)))
)

(cl:defclass ApriltagsIDsSrv-response (<ApriltagsIDsSrv-response>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <ApriltagsIDsSrv-response>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'ApriltagsIDsSrv-response)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name apriltag_finder-srv:<ApriltagsIDsSrv-response> is deprecated: use apriltag_finder-srv:ApriltagsIDsSrv-response instead.")))

(cl:ensure-generic-function 'ids-val :lambda-list '(m))
(cl:defmethod ids-val ((m <ApriltagsIDsSrv-response>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader apriltag_finder-srv:ids-val is deprecated.  Use apriltag_finder-srv:ids instead.")
  (ids m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <ApriltagsIDsSrv-response>) ostream)
  "Serializes a message object of type '<ApriltagsIDsSrv-response>"
  (cl:let ((__ros_arr_len (cl:length (cl:slot-value msg 'ids))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) __ros_arr_len) ostream))
  (cl:map cl:nil #'(cl:lambda (ele) (cl:let* ((signed ele) (unsigned (cl:if (cl:< signed 0) (cl:+ signed 4294967296) signed)))
    (cl:write-byte (cl:ldb (cl:byte 8 0) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) unsigned) ostream)
    ))
   (cl:slot-value msg 'ids))
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <ApriltagsIDsSrv-response>) istream)
  "Deserializes a message object of type '<ApriltagsIDsSrv-response>"
  (cl:let ((__ros_arr_len 0))
    (cl:setf (cl:ldb (cl:byte 8 0) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 8) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 16) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 24) __ros_arr_len) (cl:read-byte istream))
  (cl:setf (cl:slot-value msg 'ids) (cl:make-array __ros_arr_len))
  (cl:let ((vals (cl:slot-value msg 'ids)))
    (cl:dotimes (i __ros_arr_len)
    (cl:let ((unsigned 0))
      (cl:setf (cl:ldb (cl:byte 8 0) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) unsigned) (cl:read-byte istream))
      (cl:setf (cl:aref vals i) (cl:if (cl:< unsigned 2147483648) unsigned (cl:- unsigned 4294967296)))))))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<ApriltagsIDsSrv-response>)))
  "Returns string type for a service object of type '<ApriltagsIDsSrv-response>"
  "apriltag_finder/ApriltagsIDsSrvResponse")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'ApriltagsIDsSrv-response)))
  "Returns string type for a service object of type 'ApriltagsIDsSrv-response"
  "apriltag_finder/ApriltagsIDsSrvResponse")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<ApriltagsIDsSrv-response>)))
  "Returns md5sum for a message object of type '<ApriltagsIDsSrv-response>"
  "4f22efebf407aadba2ecc69df353d113")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'ApriltagsIDsSrv-response)))
  "Returns md5sum for a message object of type 'ApriltagsIDsSrv-response"
  "4f22efebf407aadba2ecc69df353d113")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<ApriltagsIDsSrv-response>)))
  "Returns full string definition for message of type '<ApriltagsIDsSrv-response>"
  (cl:format cl:nil "~%# Response~%int32[] ids~%~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'ApriltagsIDsSrv-response)))
  "Returns full string definition for message of type 'ApriltagsIDsSrv-response"
  (cl:format cl:nil "~%# Response~%int32[] ids~%~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <ApriltagsIDsSrv-response>))
  (cl:+ 0
     4 (cl:reduce #'cl:+ (cl:slot-value msg 'ids) :key #'(cl:lambda (ele) (cl:declare (cl:ignorable ele)) (cl:+ 4)))
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <ApriltagsIDsSrv-response>))
  "Converts a ROS message object to a list"
  (cl:list 'ApriltagsIDsSrv-response
    (cl:cons ':ids (ids msg))
))
(cl:defmethod roslisp-msg-protocol:service-request-type ((msg (cl:eql 'ApriltagsIDsSrv)))
  'ApriltagsIDsSrv-request)
(cl:defmethod roslisp-msg-protocol:service-response-type ((msg (cl:eql 'ApriltagsIDsSrv)))
  'ApriltagsIDsSrv-response)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'ApriltagsIDsSrv)))
  "Returns string type for a service object of type '<ApriltagsIDsSrv>"
  "apriltag_finder/ApriltagsIDsSrv")