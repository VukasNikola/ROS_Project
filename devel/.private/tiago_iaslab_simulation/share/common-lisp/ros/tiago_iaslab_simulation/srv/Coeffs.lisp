; Auto-generated. Do not edit!


(cl:in-package tiago_iaslab_simulation-srv)


;//! \htmlinclude Coeffs-request.msg.html

(cl:defclass <Coeffs-request> (roslisp-msg-protocol:ros-message)
  ((ready
    :reader ready
    :initarg :ready
    :type cl:boolean
    :initform cl:nil))
)

(cl:defclass Coeffs-request (<Coeffs-request>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <Coeffs-request>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'Coeffs-request)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name tiago_iaslab_simulation-srv:<Coeffs-request> is deprecated: use tiago_iaslab_simulation-srv:Coeffs-request instead.")))

(cl:ensure-generic-function 'ready-val :lambda-list '(m))
(cl:defmethod ready-val ((m <Coeffs-request>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader tiago_iaslab_simulation-srv:ready-val is deprecated.  Use tiago_iaslab_simulation-srv:ready instead.")
  (ready m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <Coeffs-request>) ostream)
  "Serializes a message object of type '<Coeffs-request>"
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:if (cl:slot-value msg 'ready) 1 0)) ostream)
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <Coeffs-request>) istream)
  "Deserializes a message object of type '<Coeffs-request>"
    (cl:setf (cl:slot-value msg 'ready) (cl:not (cl:zerop (cl:read-byte istream))))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<Coeffs-request>)))
  "Returns string type for a service object of type '<Coeffs-request>"
  "tiago_iaslab_simulation/CoeffsRequest")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'Coeffs-request)))
  "Returns string type for a service object of type 'Coeffs-request"
  "tiago_iaslab_simulation/CoeffsRequest")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<Coeffs-request>)))
  "Returns md5sum for a message object of type '<Coeffs-request>"
  "970e0019261c40a0b3c0c70ce65f2917")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'Coeffs-request)))
  "Returns md5sum for a message object of type 'Coeffs-request"
  "970e0019261c40a0b3c0c70ce65f2917")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<Coeffs-request>)))
  "Returns full string definition for message of type '<Coeffs-request>"
  (cl:format cl:nil "bool ready          #if true get_straaightline_node sends the coefficients' values, otherwise prints an error and kill ros~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'Coeffs-request)))
  "Returns full string definition for message of type 'Coeffs-request"
  (cl:format cl:nil "bool ready          #if true get_straaightline_node sends the coefficients' values, otherwise prints an error and kill ros~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <Coeffs-request>))
  (cl:+ 0
     1
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <Coeffs-request>))
  "Converts a ROS message object to a list"
  (cl:list 'Coeffs-request
    (cl:cons ':ready (ready msg))
))
;//! \htmlinclude Coeffs-response.msg.html

(cl:defclass <Coeffs-response> (roslisp-msg-protocol:ros-message)
  ((coeffs
    :reader coeffs
    :initarg :coeffs
    :type (cl:vector cl:float)
   :initform (cl:make-array 0 :element-type 'cl:float :initial-element 0.0)))
)

(cl:defclass Coeffs-response (<Coeffs-response>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <Coeffs-response>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'Coeffs-response)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name tiago_iaslab_simulation-srv:<Coeffs-response> is deprecated: use tiago_iaslab_simulation-srv:Coeffs-response instead.")))

(cl:ensure-generic-function 'coeffs-val :lambda-list '(m))
(cl:defmethod coeffs-val ((m <Coeffs-response>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader tiago_iaslab_simulation-srv:coeffs-val is deprecated.  Use tiago_iaslab_simulation-srv:coeffs instead.")
  (coeffs m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <Coeffs-response>) ostream)
  "Serializes a message object of type '<Coeffs-response>"
  (cl:let ((__ros_arr_len (cl:length (cl:slot-value msg 'coeffs))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) __ros_arr_len) ostream))
  (cl:map cl:nil #'(cl:lambda (ele) (cl:let ((bits (roslisp-utils:encode-single-float-bits ele)))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream)))
   (cl:slot-value msg 'coeffs))
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <Coeffs-response>) istream)
  "Deserializes a message object of type '<Coeffs-response>"
  (cl:let ((__ros_arr_len 0))
    (cl:setf (cl:ldb (cl:byte 8 0) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 8) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 16) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 24) __ros_arr_len) (cl:read-byte istream))
  (cl:setf (cl:slot-value msg 'coeffs) (cl:make-array __ros_arr_len))
  (cl:let ((vals (cl:slot-value msg 'coeffs)))
    (cl:dotimes (i __ros_arr_len)
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
    (cl:setf (cl:aref vals i) (roslisp-utils:decode-single-float-bits bits))))))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<Coeffs-response>)))
  "Returns string type for a service object of type '<Coeffs-response>"
  "tiago_iaslab_simulation/CoeffsResponse")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'Coeffs-response)))
  "Returns string type for a service object of type 'Coeffs-response"
  "tiago_iaslab_simulation/CoeffsResponse")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<Coeffs-response>)))
  "Returns md5sum for a message object of type '<Coeffs-response>"
  "970e0019261c40a0b3c0c70ce65f2917")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'Coeffs-response)))
  "Returns md5sum for a message object of type 'Coeffs-response"
  "970e0019261c40a0b3c0c70ce65f2917")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<Coeffs-response>)))
  "Returns full string definition for message of type '<Coeffs-response>"
  (cl:format cl:nil "float32[] coeffs~%~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'Coeffs-response)))
  "Returns full string definition for message of type 'Coeffs-response"
  (cl:format cl:nil "float32[] coeffs~%~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <Coeffs-response>))
  (cl:+ 0
     4 (cl:reduce #'cl:+ (cl:slot-value msg 'coeffs) :key #'(cl:lambda (ele) (cl:declare (cl:ignorable ele)) (cl:+ 4)))
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <Coeffs-response>))
  "Converts a ROS message object to a list"
  (cl:list 'Coeffs-response
    (cl:cons ':coeffs (coeffs msg))
))
(cl:defmethod roslisp-msg-protocol:service-request-type ((msg (cl:eql 'Coeffs)))
  'Coeffs-request)
(cl:defmethod roslisp-msg-protocol:service-response-type ((msg (cl:eql 'Coeffs)))
  'Coeffs-response)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'Coeffs)))
  "Returns string type for a service object of type '<Coeffs>"
  "tiago_iaslab_simulation/Coeffs")