;; Auto-generated. Do not edit!


(when (boundp 'assignment2_package::GetObjectPose)
  (if (not (find-package "ASSIGNMENT2_PACKAGE"))
    (make-package "ASSIGNMENT2_PACKAGE"))
  (shadow 'GetObjectPose (find-package "ASSIGNMENT2_PACKAGE")))
(unless (find-package "ASSIGNMENT2_PACKAGE::GETOBJECTPOSE")
  (make-package "ASSIGNMENT2_PACKAGE::GETOBJECTPOSE"))
(unless (find-package "ASSIGNMENT2_PACKAGE::GETOBJECTPOSEREQUEST")
  (make-package "ASSIGNMENT2_PACKAGE::GETOBJECTPOSEREQUEST"))
(unless (find-package "ASSIGNMENT2_PACKAGE::GETOBJECTPOSERESPONSE")
  (make-package "ASSIGNMENT2_PACKAGE::GETOBJECTPOSERESPONSE"))

(in-package "ROS")



(if (not (find-package "GEOMETRY_MSGS"))
  (ros::roseus-add-msgs "geometry_msgs"))


(defclass assignment2_package::GetObjectPoseRequest
  :super ros::object
  :slots ())

(defmethod assignment2_package::GetObjectPoseRequest
  (:init
   (&key
    )
   (send-super :init)
   self)
  (:serialization-length
   ()
   (+
    0
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;;
   self)
  )

(defclass assignment2_package::GetObjectPoseResponse
  :super ros::object
  :slots (_obj_id _obj_pose ))

(defmethod assignment2_package::GetObjectPoseResponse
  (:init
   (&key
    ((:obj_id __obj_id) 0)
    ((:obj_pose __obj_pose) (instance geometry_msgs::PoseStamped :init))
    )
   (send-super :init)
   (setq _obj_id (round __obj_id))
   (setq _obj_pose __obj_pose)
   self)
  (:obj_id
   (&optional __obj_id)
   (if __obj_id (setq _obj_id __obj_id)) _obj_id)
  (:obj_pose
   (&rest __obj_pose)
   (if (keywordp (car __obj_pose))
       (send* _obj_pose __obj_pose)
     (progn
       (if __obj_pose (setq _obj_pose (car __obj_pose)))
       _obj_pose)))
  (:serialization-length
   ()
   (+
    ;; uint32 _obj_id
    4
    ;; geometry_msgs/PoseStamped _obj_pose
    (send _obj_pose :serialization-length)
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; uint32 _obj_id
       (write-long _obj_id s)
     ;; geometry_msgs/PoseStamped _obj_pose
       (send _obj_pose :serialize s)
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; uint32 _obj_id
     (setq _obj_id (sys::peek buf ptr- :integer)) (incf ptr- 4)
   ;; geometry_msgs/PoseStamped _obj_pose
     (send _obj_pose :deserialize buf ptr-) (incf ptr- (send _obj_pose :serialization-length))
   ;;
   self)
  )

(defclass assignment2_package::GetObjectPose
  :super ros::object
  :slots ())

(setf (get assignment2_package::GetObjectPose :md5sum-) "d6d4f3bebbf31073267f6fff2a596a9a")
(setf (get assignment2_package::GetObjectPose :datatype-) "assignment2_package/GetObjectPose")
(setf (get assignment2_package::GetObjectPose :request) assignment2_package::GetObjectPoseRequest)
(setf (get assignment2_package::GetObjectPose :response) assignment2_package::GetObjectPoseResponse)

(defmethod assignment2_package::GetObjectPoseRequest
  (:response () (instance assignment2_package::GetObjectPoseResponse :init)))

(setf (get assignment2_package::GetObjectPoseRequest :md5sum-) "d6d4f3bebbf31073267f6fff2a596a9a")
(setf (get assignment2_package::GetObjectPoseRequest :datatype-) "assignment2_package/GetObjectPoseRequest")
(setf (get assignment2_package::GetObjectPoseRequest :definition-)
      "#No request
---
uint32 obj_id
geometry_msgs/PoseStamped obj_pose

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
")

(setf (get assignment2_package::GetObjectPoseResponse :md5sum-) "d6d4f3bebbf31073267f6fff2a596a9a")
(setf (get assignment2_package::GetObjectPoseResponse :datatype-) "assignment2_package/GetObjectPoseResponse")
(setf (get assignment2_package::GetObjectPoseResponse :definition-)
      "#No request
---
uint32 obj_id
geometry_msgs/PoseStamped obj_pose

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
")



(provide :assignment2_package/GetObjectPose "d6d4f3bebbf31073267f6fff2a596a9a")


