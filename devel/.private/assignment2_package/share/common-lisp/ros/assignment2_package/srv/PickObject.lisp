; Auto-generated. Do not edit!


(cl:in-package assignment2_package-srv)


;//! \htmlinclude PickObject-request.msg.html

(cl:defclass <PickObject-request> (roslisp-msg-protocol:ros-message)
  ()
)

(cl:defclass PickObject-request (<PickObject-request>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <PickObject-request>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'PickObject-request)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name assignment2_package-srv:<PickObject-request> is deprecated: use assignment2_package-srv:PickObject-request instead.")))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <PickObject-request>) ostream)
  "Serializes a message object of type '<PickObject-request>"
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <PickObject-request>) istream)
  "Deserializes a message object of type '<PickObject-request>"
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
  "937c9679a518e3a18d831e57125ea522")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'PickObject-request)))
  "Returns md5sum for a message object of type 'PickObject-request"
  "937c9679a518e3a18d831e57125ea522")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<PickObject-request>)))
  "Returns full string definition for message of type '<PickObject-request>"
  (cl:format cl:nil "# PickObject.srv~%# (Empty request)~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'PickObject-request)))
  "Returns full string definition for message of type 'PickObject-request"
  (cl:format cl:nil "# PickObject.srv~%# (Empty request)~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <PickObject-request>))
  (cl:+ 0
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <PickObject-request>))
  "Converts a ROS message object to a list"
  (cl:list 'PickObject-request
))
;//! \htmlinclude PickObject-response.msg.html

(cl:defclass <PickObject-response> (roslisp-msg-protocol:ros-message)
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

(cl:ensure-generic-function 'message-val :lambda-list '(m))
(cl:defmethod message-val ((m <PickObject-response>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader assignment2_package-srv:message-val is deprecated.  Use assignment2_package-srv:message instead.")
  (message m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <PickObject-response>) ostream)
  "Serializes a message object of type '<PickObject-response>"
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:if (cl:slot-value msg 'success) 1 0)) ostream)
  (cl:let ((__ros_str_len (cl:length (cl:slot-value msg 'message))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) __ros_str_len) ostream))
  (cl:map cl:nil #'(cl:lambda (c) (cl:write-byte (cl:char-code c) ostream)) (cl:slot-value msg 'message))
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <PickObject-response>) istream)
  "Deserializes a message object of type '<PickObject-response>"
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
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<PickObject-response>)))
  "Returns string type for a service object of type '<PickObject-response>"
  "assignment2_package/PickObjectResponse")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'PickObject-response)))
  "Returns string type for a service object of type 'PickObject-response"
  "assignment2_package/PickObjectResponse")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<PickObject-response>)))
  "Returns md5sum for a message object of type '<PickObject-response>"
  "937c9679a518e3a18d831e57125ea522")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'PickObject-response)))
  "Returns md5sum for a message object of type 'PickObject-response"
  "937c9679a518e3a18d831e57125ea522")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<PickObject-response>)))
  "Returns full string definition for message of type '<PickObject-response>"
  (cl:format cl:nil "bool success~%string message~%~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'PickObject-response)))
  "Returns full string definition for message of type 'PickObject-response"
  (cl:format cl:nil "bool success~%string message~%~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <PickObject-response>))
  (cl:+ 0
     1
     4 (cl:length (cl:slot-value msg 'message))
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <PickObject-response>))
  "Converts a ROS message object to a list"
  (cl:list 'PickObject-response
    (cl:cons ':success (success msg))
    (cl:cons ':message (message msg))
))
(cl:defmethod roslisp-msg-protocol:service-request-type ((msg (cl:eql 'PickObject)))
  'PickObject-request)
(cl:defmethod roslisp-msg-protocol:service-response-type ((msg (cl:eql 'PickObject)))
  'PickObject-response)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'PickObject)))
  "Returns string type for a service object of type '<PickObject>"
  "assignment2_package/PickObject")