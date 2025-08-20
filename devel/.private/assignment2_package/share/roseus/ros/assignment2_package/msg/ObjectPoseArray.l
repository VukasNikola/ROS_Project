;; Auto-generated. Do not edit!


(when (boundp 'assignment2_package::ObjectPoseArray)
  (if (not (find-package "ASSIGNMENT2_PACKAGE"))
    (make-package "ASSIGNMENT2_PACKAGE"))
  (shadow 'ObjectPoseArray (find-package "ASSIGNMENT2_PACKAGE")))
(unless (find-package "ASSIGNMENT2_PACKAGE::OBJECTPOSEARRAY")
  (make-package "ASSIGNMENT2_PACKAGE::OBJECTPOSEARRAY"))

(in-package "ROS")
;;//! \htmlinclude ObjectPoseArray.msg.html


(defclass assignment2_package::ObjectPoseArray
  :super ros::object
  :slots (_objects ))

(defmethod assignment2_package::ObjectPoseArray
  (:init
   (&key
    ((:objects __objects) ())
    )
   (send-super :init)
   (setq _objects __objects)
   self)
  (:objects
   (&rest __objects)
   (if (keywordp (car __objects))
       (send* _objects __objects)
     (progn
       (if __objects (setq _objects (car __objects)))
       _objects)))
  (:serialization-length
   ()
   (+
    ;; assignment2_package/ObjectPose[] _objects
    (apply #'+ (send-all _objects :serialization-length)) 4
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; assignment2_package/ObjectPose[] _objects
     (write-long (length _objects) s)
     (dolist (elem _objects)
       (send elem :serialize s)
       )
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; assignment2_package/ObjectPose[] _objects
   (let (n)
     (setq n (sys::peek buf ptr- :integer)) (incf ptr- 4)
     (setq _objects (let (r) (dotimes (i n) (push (instance assignment2_package::ObjectPose :init) r)) r))
     (dolist (elem- _objects)
     (send elem- :deserialize buf ptr-) (incf ptr- (send elem- :serialization-length))
     ))
   ;;
   self)
  )

(setf (get assignment2_package::ObjectPoseArray :md5sum-) "5d14148e7c4ffcd67a5e7838038a1331")
(setf (get assignment2_package::ObjectPoseArray :datatype-) "assignment2_package/ObjectPoseArray")
(setf (get assignment2_package::ObjectPoseArray :definition-)
      "assignment2_package/ObjectPose[] objects
================================================================================
MSG: assignment2_package/ObjectPose
uint32 id
geometry_msgs/PoseStamped pose

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



(provide :assignment2_package/ObjectPoseArray "5d14148e7c4ffcd67a5e7838038a1331")


