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





(defclass assignment2_package::PlaceObjectRequest
  :super ros::object
  :slots (_target_id ))

(defmethod assignment2_package::PlaceObjectRequest
  (:init
   (&key
    ((:target_id __target_id) 0)
    )
   (send-super :init)
   (setq _target_id (round __target_id))
   self)
  (:target_id
   (&optional __target_id)
   (if __target_id (setq _target_id __target_id)) _target_id)
  (:serialization-length
   ()
   (+
    ;; uint32 _target_id
    4
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; uint32 _target_id
       (write-long _target_id s)
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; uint32 _target_id
     (setq _target_id (sys::peek buf ptr- :integer)) (incf ptr- 4)
   ;;
   self)
  )

(defclass assignment2_package::PlaceObjectResponse
  :super ros::object
  :slots (_success ))

(defmethod assignment2_package::PlaceObjectResponse
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

(defclass assignment2_package::PlaceObject
  :super ros::object
  :slots ())

(setf (get assignment2_package::PlaceObject :md5sum-) "8a0de6da721abc6c98db8ab55063d9c6")
(setf (get assignment2_package::PlaceObject :datatype-) "assignment2_package/PlaceObject")
(setf (get assignment2_package::PlaceObject :request) assignment2_package::PlaceObjectRequest)
(setf (get assignment2_package::PlaceObject :response) assignment2_package::PlaceObjectResponse)

(defmethod assignment2_package::PlaceObjectRequest
  (:response () (instance assignment2_package::PlaceObjectResponse :init)))

(setf (get assignment2_package::PlaceObjectRequest :md5sum-) "8a0de6da721abc6c98db8ab55063d9c6")
(setf (get assignment2_package::PlaceObjectRequest :datatype-) "assignment2_package/PlaceObjectRequest")
(setf (get assignment2_package::PlaceObjectRequest :definition-)
      "uint32 target_id
---
bool success
")

(setf (get assignment2_package::PlaceObjectResponse :md5sum-) "8a0de6da721abc6c98db8ab55063d9c6")
(setf (get assignment2_package::PlaceObjectResponse :datatype-) "assignment2_package/PlaceObjectResponse")
(setf (get assignment2_package::PlaceObjectResponse :definition-)
      "uint32 target_id
---
bool success
")



(provide :assignment2_package/PlaceObject "8a0de6da721abc6c98db8ab55063d9c6")


