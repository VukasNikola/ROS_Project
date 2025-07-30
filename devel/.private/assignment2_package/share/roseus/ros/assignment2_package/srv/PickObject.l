;; Auto-generated. Do not edit!


(when (boundp 'assignment2_package::PickObject)
  (if (not (find-package "ASSIGNMENT2_PACKAGE"))
    (make-package "ASSIGNMENT2_PACKAGE"))
  (shadow 'PickObject (find-package "ASSIGNMENT2_PACKAGE")))
(unless (find-package "ASSIGNMENT2_PACKAGE::PICKOBJECT")
  (make-package "ASSIGNMENT2_PACKAGE::PICKOBJECT"))
(unless (find-package "ASSIGNMENT2_PACKAGE::PICKOBJECTREQUEST")
  (make-package "ASSIGNMENT2_PACKAGE::PICKOBJECTREQUEST"))
(unless (find-package "ASSIGNMENT2_PACKAGE::PICKOBJECTRESPONSE")
  (make-package "ASSIGNMENT2_PACKAGE::PICKOBJECTRESPONSE"))

(in-package "ROS")

(if (not (find-package "GEOMETRY_MSGS"))
  (ros::roseus-add-msgs "geometry_msgs"))




(defclass assignment2_package::PickObjectRequest
  :super ros::object
  :slots (_target ))

(defmethod assignment2_package::PickObjectRequest
  (:init
   (&key
    ((:target __target) (instance geometry_msgs::PoseStamped :init))
    )
   (send-super :init)
   (setq _target __target)
   self)
  (:target
   (&rest __target)
   (if (keywordp (car __target))
       (send* _target __target)
     (progn
       (if __target (setq _target (car __target)))
       _target)))
  (:serialization-length
   ()
   (+
    ;; geometry_msgs/PoseStamped _target
    (send _target :serialization-length)
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; geometry_msgs/PoseStamped _target
       (send _target :serialize s)
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; geometry_msgs/PoseStamped _target
     (send _target :deserialize buf ptr-) (incf ptr- (send _target :serialization-length))
   ;;
   self)
  )

(defclass assignment2_package::PickObjectResponse
  :super ros::object
  :slots (_success ))

(defmethod assignment2_package::PickObjectResponse
  (:init
   (&key
    ((:success __success) nil)
    )
   (send-super :init)
   (setq _success __success)
   self)
  (:success
   (&optional (__success :null))
   (if (not (eq __success :null)) (setq _success __success)) _success)
  (:serialization-length
   ()
   (+
    ;; bool _success
    1
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; bool _success
       (if _success (write-byte -1 s) (write-byte 0 s))
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; bool _success
     (setq _success (not (= 0 (sys::peek buf ptr- :char)))) (incf ptr- 1)
   ;;
   self)
  )

(defclass assignment2_package::PickObject
  :super ros::object
  :slots ())

(setf (get assignment2_package::PickObject :md5sum-) "327c2f6cdc43d58ff36d1dc7acc1780f")
(setf (get assignment2_package::PickObject :datatype-) "assignment2_package/PickObject")
(setf (get assignment2_package::PickObject :request) assignment2_package::PickObjectRequest)
(setf (get assignment2_package::PickObject :response) assignment2_package::PickObjectResponse)

(defmethod assignment2_package::PickObjectRequest
  (:response () (instance assignment2_package::PickObjectResponse :init)))

(setf (get assignment2_package::PickObjectRequest :md5sum-) "327c2f6cdc43d58ff36d1dc7acc1780f")
(setf (get assignment2_package::PickObjectRequest :datatype-) "assignment2_package/PickObjectRequest")
(setf (get assignment2_package::PickObjectRequest :definition-)
      "geometry_msgs/PoseStamped target

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
")

(setf (get assignment2_package::PickObjectResponse :md5sum-) "327c2f6cdc43d58ff36d1dc7acc1780f")
(setf (get assignment2_package::PickObjectResponse :datatype-) "assignment2_package/PickObjectResponse")
(setf (get assignment2_package::PickObjectResponse :definition-)
      "geometry_msgs/PoseStamped target

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
")



(provide :assignment2_package/PickObject "327c2f6cdc43d58ff36d1dc7acc1780f")


