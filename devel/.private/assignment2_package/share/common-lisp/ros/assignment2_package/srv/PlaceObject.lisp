; Auto-generated. Do not edit!


(cl:in-package assignment2_package-srv)


;//! \htmlinclude PlaceObject-request.msg.html

(cl:defclass <PlaceObject-request> (roslisp-msg-protocol:ros-message)
  ((target_id
    :reader target_id
    :initarg :target_id
    :type cl:integer
    :initform 0))
)

(cl:defclass PlaceObject-request (<PlaceObject-request>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <PlaceObject-request>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'PlaceObject-request)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name assignment2_package-srv:<PlaceObject-request> is deprecated: use assignment2_package-srv:PlaceObject-request instead.")))

(cl:ensure-generic-function 'target_id-val :lambda-list '(m))
(cl:defmethod target_id-val ((m <PlaceObject-request>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader assignment2_package-srv:target_id-val is deprecated.  Use assignment2_package-srv:target_id instead.")
  (target_id m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <PlaceObject-request>) ostream)
  "Serializes a message object of type '<PlaceObject-request>"
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'target_id)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 8) (cl:slot-value msg 'target_id)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 16) (cl:slot-value msg 'target_id)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 24) (cl:slot-value msg 'target_id)) ostream)
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <PlaceObject-request>) istream)
  "Deserializes a message object of type '<PlaceObject-request>"
    (cl:setf (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'target_id)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 8) (cl:slot-value msg 'target_id)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 16) (cl:slot-value msg 'target_id)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 24) (cl:slot-value msg 'target_id)) (cl:read-byte istream))
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
  "8a0de6da721abc6c98db8ab55063d9c6")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'PlaceObject-request)))
  "Returns md5sum for a message object of type 'PlaceObject-request"
  "8a0de6da721abc6c98db8ab55063d9c6")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<PlaceObject-request>)))
  "Returns full string definition for message of type '<PlaceObject-request>"
  (cl:format cl:nil "uint32 target_id~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'PlaceObject-request)))
  "Returns full string definition for message of type 'PlaceObject-request"
  (cl:format cl:nil "uint32 target_id~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <PlaceObject-request>))
  (cl:+ 0
     4
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <PlaceObject-request>))
  "Converts a ROS message object to a list"
  (cl:list 'PlaceObject-request
    (cl:cons ':target_id (target_id msg))
))
;//! \htmlinclude PlaceObject-response.msg.html

(cl:defclass <PlaceObject-response> (roslisp-msg-protocol:ros-message)
  ((success
    :reader success
    :initarg :success
    :type cl:boolean
    :initform cl:nil))
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
(cl:defmethod roslisp-msg-protocol:serialize ((msg <PlaceObject-response>) ostream)
  "Serializes a message object of type '<PlaceObject-response>"
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:if (cl:slot-value msg 'success) 1 0)) ostream)
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <PlaceObject-response>) istream)
  "Deserializes a message object of type '<PlaceObject-response>"
    (cl:setf (cl:slot-value msg 'success) (cl:not (cl:zerop (cl:read-byte istream))))
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
  "8a0de6da721abc6c98db8ab55063d9c6")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'PlaceObject-response)))
  "Returns md5sum for a message object of type 'PlaceObject-response"
  "8a0de6da721abc6c98db8ab55063d9c6")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<PlaceObject-response>)))
  "Returns full string definition for message of type '<PlaceObject-response>"
  (cl:format cl:nil "bool success~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'PlaceObject-response)))
  "Returns full string definition for message of type 'PlaceObject-response"
  (cl:format cl:nil "bool success~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <PlaceObject-response>))
  (cl:+ 0
     1
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <PlaceObject-response>))
  "Converts a ROS message object to a list"
  (cl:list 'PlaceObject-response
    (cl:cons ':success (success msg))
))
(cl:defmethod roslisp-msg-protocol:service-request-type ((msg (cl:eql 'PlaceObject)))
  'PlaceObject-request)
(cl:defmethod roslisp-msg-protocol:service-response-type ((msg (cl:eql 'PlaceObject)))
  'PlaceObject-response)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'PlaceObject)))
  "Returns string type for a service object of type '<PlaceObject>"
  "assignment2_package/PlaceObject")