;; Auto-generated. Do not edit!


(when (boundp 'assignment2_package::PlaceObject)
  (if (not (find-package "ASSIGNMENT2_PACKAGE"))
    (make-package "ASSIGNMENT2_PACKAGE"))
  (shadow 'PlaceObject (find-package "ASSIGNMENT2_PACKAGE")))
(unless (find-package "ASSIGNMENT2_PACKAGE::PLACEOBJECT")
  (make-package "ASSIGNMENT2_PACKAGE::PLACEOBJECT"))
(unless (find-package "ASSIGNMENT2_PACKAGE::PLACEOBJECTREQUEST")
  (make-package "ASSIGNMENT2_PACKAGE::PLACEOBJECTREQUEST"))
(unless (find-package "ASSIGNMENT2_PACKAGE::PLACEOBJECTRESPONSE")
  (make-package "ASSIGNMENT2_PACKAGE::PLACEOBJECTRESPONSE"))

(in-package "ROS")

(if (not (find-package "GEOMETRY_MSGS"))
  (ros::roseus-add-msgs "geometry_msgs"))




(defclass assignment2_package::PlaceObjectRequest
  :super ros::object
  :slots (_target_pose ))

(defmethod assignment2_package::PlaceObjectRequest
  (:init
   (&key
    ((:target_pose __target_pose) (instance geometry_msgs::PoseStamped :init))
    )
   (send-super :init)
   (setq _target_pose __target_pose)
   self)
  (:target_pose
   (&rest __target_pose)
   (if (keywordp (car __target_pose))
       (send* _target_pose __target_pose)
     (progn
       (if __target_pose (setq _target_pose (car __target_pose)))
       _target_pose)))
  (:serialization-length
   ()
   (+
    ;; geometry_msgs/PoseStamped _target_pose
    (send _target_pose :serialization-length)
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; geometry_msgs/PoseStamped _target_pose
       (send _target_pose :serialize s)
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; geometry_msgs/PoseStamped _target_pose
     (send _target_pose :deserialize buf ptr-) (incf ptr- (send _target_pose :serialization-length))
   ;;
   self)
  )

(defclass assignment2_package::PlaceObjectResponse
  :super ros::object
  :slots (_success _message ))

(defmethod assignment2_package::PlaceObjectResponse
  (:init
   (&key
    ((:success __success) nil)
    ((:message __message) "")
    )
   (send-super :init)
   (setq _success __success)
   (setq _message (string __message))
   self)
  (:success
   (&optional (__success :null))
   (if (not (eq __success :null)) (setq _success __success)) _success)
  (:message
   (&optional __message)
   (if __message (setq _message __message)) _message)
  (:serialization-length
   ()
   (+
    ;; bool _success
    1
    ;; string _message
    4 (length _message)
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; bool _success
       (if _success (write-byte -1 s) (write-byte 0 s))
     ;; string _message
       (write-long (length _message) s) (princ _message s)
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; bool _success
     (setq _success (not (= 0 (sys::peek buf ptr- :char)))) (incf ptr- 1)
   ;; string _message
     (let (n) (setq n (sys::peek buf ptr- :integer)) (incf ptr- 4) (setq _message (subseq buf ptr- (+ ptr- n))) (incf ptr- n))
   ;;
   self)
  )

(defclass assignment2_package::PlaceObject
  :super ros::object
  :slots ())

(setf (get assignment2_package::PlaceObject :md5sum-) "9a1b28e959609263fbb3c7ffb98e17db")
(setf (get assignment2_package::PlaceObject :datatype-) "assignment2_package/PlaceObject")
(setf (get assignment2_package::PlaceObject :request) assignment2_package::PlaceObjectRequest)
(setf (get assignment2_package::PlaceObject :response) assignment2_package::PlaceObjectResponse)

(defmethod assignment2_package::PlaceObjectRequest
  (:response () (instance assignment2_package::PlaceObjectResponse :init)))

(setf (get assignment2_package::PlaceObjectRequest :md5sum-) "9a1b28e959609263fbb3c7ffb98e17db")
(setf (get assignment2_package::PlaceObjectRequest :datatype-) "assignment2_package/PlaceObjectRequest")
(setf (get assignment2_package::PlaceObjectRequest :definition-)
      "# PlaceObject.srv
geometry_msgs/PoseStamped target_pose

================================================================================
MSG: geometry_msgs/PoseStamped
# A Pose with reference coordinate frame and timestamp
Header header
Pose pose

================================================================================
MSG: std_msgs/Header
# Standard metadata for higher-level stamped data types.
# This is generally used to communicate timestamped data 
# in a particular coordinate frame.
# 
# sequence ID: consecutively increasing ID 
uint32 seq
#Two-integer timestamp that is expressed as:
# * stamp.sec: seconds (stamp_secs) since epoch (in Python the variable is called 'secs')
# * stamp.nsec: nanoseconds since stamp_secs (in Python the variable is called 'nsecs')
# time-handling sugar is provided by the client library
time stamp
#Frame this data is associated with
string frame_id

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
---
bool success
string message

")

(setf (get assignment2_package::PlaceObjectResponse :md5sum-) "9a1b28e959609263fbb3c7ffb98e17db")
(setf (get assignment2_package::PlaceObjectResponse :datatype-) "assignment2_package/PlaceObjectResponse")
(setf (get assignment2_package::PlaceObjectResponse :definition-)
      "# PlaceObject.srv
geometry_msgs/PoseStamped target_pose

================================================================================
MSG: geometry_msgs/PoseStamped
# A Pose with reference coordinate frame and timestamp
Header header
Pose pose

================================================================================
MSG: std_msgs/Header
# Standard metadata for higher-level stamped data types.
# This is generally used to communicate timestamped data 
# in a particular coordinate frame.
# 
# sequence ID: consecutively increasing ID 
uint32 seq
#Two-integer timestamp that is expressed as:
# * stamp.sec: seconds (stamp_secs) since epoch (in Python the variable is called 'secs')
# * stamp.nsec: nanoseconds since stamp_secs (in Python the variable is called 'nsecs')
# time-handling sugar is provided by the client library
time stamp
#Frame this data is associated with
string frame_id

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
---
bool success
string message

")



(provide :assignment2_package/PlaceObject "9a1b28e959609263fbb3c7ffb98e17db")


