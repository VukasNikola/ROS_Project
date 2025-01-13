;; Auto-generated. Do not edit!


(when (boundp 'apriltag_finder::ApriltagsIDsSrv)
  (if (not (find-package "APRILTAG_FINDER"))
    (make-package "APRILTAG_FINDER"))
  (shadow 'ApriltagsIDsSrv (find-package "APRILTAG_FINDER")))
(unless (find-package "APRILTAG_FINDER::APRILTAGSIDSSRV")
  (make-package "APRILTAG_FINDER::APRILTAGSIDSSRV"))
(unless (find-package "APRILTAG_FINDER::APRILTAGSIDSSRVREQUEST")
  (make-package "APRILTAG_FINDER::APRILTAGSIDSSRVREQUEST"))
(unless (find-package "APRILTAG_FINDER::APRILTAGSIDSSRVRESPONSE")
  (make-package "APRILTAG_FINDER::APRILTAGSIDSSRVRESPONSE"))

(in-package "ROS")





(defclass apriltag_finder::ApriltagsIDsSrvRequest
  :super ros::object
  :slots ())

(defmethod apriltag_finder::ApriltagsIDsSrvRequest
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

(defclass apriltag_finder::ApriltagsIDsSrvResponse
  :super ros::object
  :slots (_ids ))

(defmethod apriltag_finder::ApriltagsIDsSrvResponse
  (:init
   (&key
    ((:ids __ids) (make-array 0 :initial-element 0 :element-type :integer))
    )
   (send-super :init)
   (setq _ids __ids)
   self)
  (:ids
   (&optional __ids)
   (if __ids (setq _ids __ids)) _ids)
  (:serialization-length
   ()
   (+
    ;; int32[] _ids
    (* 4    (length _ids)) 4
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; int32[] _ids
     (write-long (length _ids) s)
     (dotimes (i (length _ids))
       (write-long (elt _ids i) s)
       )
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; int32[] _ids
   (let (n)
     (setq n (sys::peek buf ptr- :integer)) (incf ptr- 4)
     (setq _ids (instantiate integer-vector n))
     (dotimes (i n)
     (setf (elt _ids i) (sys::peek buf ptr- :integer)) (incf ptr- 4)
     ))
   ;;
   self)
  )

(defclass apriltag_finder::ApriltagsIDsSrv
  :super ros::object
  :slots ())

(setf (get apriltag_finder::ApriltagsIDsSrv :md5sum-) "4f22efebf407aadba2ecc69df353d113")
(setf (get apriltag_finder::ApriltagsIDsSrv :datatype-) "apriltag_finder/ApriltagsIDsSrv")
(setf (get apriltag_finder::ApriltagsIDsSrv :request) apriltag_finder::ApriltagsIDsSrvRequest)
(setf (get apriltag_finder::ApriltagsIDsSrv :response) apriltag_finder::ApriltagsIDsSrvResponse)

(defmethod apriltag_finder::ApriltagsIDsSrvRequest
  (:response () (instance apriltag_finder::ApriltagsIDsSrvResponse :init)))

(setf (get apriltag_finder::ApriltagsIDsSrvRequest :md5sum-) "4f22efebf407aadba2ecc69df353d113")
(setf (get apriltag_finder::ApriltagsIDsSrvRequest :datatype-) "apriltag_finder/ApriltagsIDsSrvRequest")
(setf (get apriltag_finder::ApriltagsIDsSrvRequest :definition-)
      "# Request
---

# Response
int32[] ids

")

(setf (get apriltag_finder::ApriltagsIDsSrvResponse :md5sum-) "4f22efebf407aadba2ecc69df353d113")
(setf (get apriltag_finder::ApriltagsIDsSrvResponse :datatype-) "apriltag_finder/ApriltagsIDsSrvResponse")
(setf (get apriltag_finder::ApriltagsIDsSrvResponse :definition-)
      "# Request
---

# Response
int32[] ids

")



(provide :apriltag_finder/ApriltagsIDsSrv "4f22efebf407aadba2ecc69df353d113")


