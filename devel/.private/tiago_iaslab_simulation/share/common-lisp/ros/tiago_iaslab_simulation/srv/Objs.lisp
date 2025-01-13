; Auto-generated. Do not edit!


(cl:in-package tiago_iaslab_simulation-srv)


;//! \htmlinclude Objs-request.msg.html

(cl:defclass <Objs-request> (roslisp-msg-protocol:ros-message)
  ((ready
    :reader ready
    :initarg :ready
    :type cl:boolean
    :initform cl:nil))
)

(cl:defclass Objs-request (<Objs-request>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <Objs-request>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'Objs-request)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name tiago_iaslab_simulation-srv:<Objs-request> is deprecated: use tiago_iaslab_simulation-srv:Objs-request instead.")))

(cl:ensure-generic-function 'ready-val :lambda-list '(m))
(cl:defmethod ready-val ((m <Objs-request>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader tiago_iaslab_simulation-srv:ready-val is deprecated.  Use tiago_iaslab_simulation-srv:ready instead.")
  (ready m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <Objs-request>) ostream)
  "Serializes a message object of type '<Objs-request>"
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:if (cl:slot-value msg 'ready) 1 0)) ostream)
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <Objs-request>) istream)
  "Deserializes a message object of type '<Objs-request>"
    (cl:setf (cl:slot-value msg 'ready) (cl:not (cl:zerop (cl:read-byte istream))))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<Objs-request>)))
  "Returns string type for a service object of type '<Objs-request>"
  "tiago_iaslab_simulation/ObjsRequest")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'Objs-request)))
  "Returns string type for a service object of type 'Objs-request"
  "tiago_iaslab_simulation/ObjsRequest")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<Objs-request>)))
  "Returns md5sum for a message object of type '<Objs-request>"
  "b0d667d823cb9909fa2a290df26f97da")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'Objs-request)))
  "Returns md5sum for a message object of type 'Objs-request"
  "b0d667d823cb9909fa2a290df26f97da")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<Objs-request>)))
  "Returns full string definition for message of type '<Objs-request>"
  (cl:format cl:nil "bool ready          #if true apriltag_ids sends the apriltags IDs, otherwise prints an error and kill ros~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'Objs-request)))
  "Returns full string definition for message of type 'Objs-request"
  (cl:format cl:nil "bool ready          #if true apriltag_ids sends the apriltags IDs, otherwise prints an error and kill ros~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <Objs-request>))
  (cl:+ 0
     1
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <Objs-request>))
  "Converts a ROS message object to a list"
  (cl:list 'Objs-request
    (cl:cons ':ready (ready msg))
))
;//! \htmlinclude Objs-response.msg.html

(cl:defclass <Objs-response> (roslisp-msg-protocol:ros-message)
  ((ids
    :reader ids
    :initarg :ids
    :type (cl:vector cl:integer)
   :initform (cl:make-array 0 :element-type 'cl:integer :initial-element 0)))
)

(cl:defclass Objs-response (<Objs-response>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <Objs-response>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'Objs-response)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name tiago_iaslab_simulation-srv:<Objs-response> is deprecated: use tiago_iaslab_simulation-srv:Objs-response instead.")))

(cl:ensure-generic-function 'ids-val :lambda-list '(m))
(cl:defmethod ids-val ((m <Objs-response>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader tiago_iaslab_simulation-srv:ids-val is deprecated.  Use tiago_iaslab_simulation-srv:ids instead.")
  (ids m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <Objs-response>) ostream)
  "Serializes a message object of type '<Objs-response>"
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
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <Objs-response>) istream)
  "Deserializes a message object of type '<Objs-response>"
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
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<Objs-response>)))
  "Returns string type for a service object of type '<Objs-response>"
  "tiago_iaslab_simulation/ObjsResponse")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'Objs-response)))
  "Returns string type for a service object of type 'Objs-response"
  "tiago_iaslab_simulation/ObjsResponse")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<Objs-response>)))
  "Returns md5sum for a message object of type '<Objs-response>"
  "b0d667d823cb9909fa2a290df26f97da")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'Objs-response)))
  "Returns md5sum for a message object of type 'Objs-response"
  "b0d667d823cb9909fa2a290df26f97da")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<Objs-response>)))
  "Returns full string definition for message of type '<Objs-response>"
  (cl:format cl:nil "int32[] ids~%~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'Objs-response)))
  "Returns full string definition for message of type 'Objs-response"
  (cl:format cl:nil "int32[] ids~%~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <Objs-response>))
  (cl:+ 0
     4 (cl:reduce #'cl:+ (cl:slot-value msg 'ids) :key #'(cl:lambda (ele) (cl:declare (cl:ignorable ele)) (cl:+ 4)))
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <Objs-response>))
  "Converts a ROS message object to a list"
  (cl:list 'Objs-response
    (cl:cons ':ids (ids msg))
))
(cl:defmethod roslisp-msg-protocol:service-request-type ((msg (cl:eql 'Objs)))
  'Objs-request)
(cl:defmethod roslisp-msg-protocol:service-response-type ((msg (cl:eql 'Objs)))
  'Objs-response)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'Objs)))
  "Returns string type for a service object of type '<Objs>"
  "tiago_iaslab_simulation/Objs")