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





(defclass assignment2_package::PickObjectRequest
  :super ros::object
  :slots ())

(defmethod assignment2_package::PickObjectRequest
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

(defclass assignment2_package::PickObjectResponse
  :super ros::object
  :slots (_success _message ))

(defmethod assignment2_package::PickObjectResponse
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

(defclass assignment2_package::PickObject
  :super ros::object
  :slots ())

(setf (get assignment2_package::PickObject :md5sum-) "937c9679a518e3a18d831e57125ea522")
(setf (get assignment2_package::PickObject :datatype-) "assignment2_package/PickObject")
(setf (get assignment2_package::PickObject :request) assignment2_package::PickObjectRequest)
(setf (get assignment2_package::PickObject :response) assignment2_package::PickObjectResponse)

(defmethod assignment2_package::PickObjectRequest
  (:response () (instance assignment2_package::PickObjectResponse :init)))

(setf (get assignment2_package::PickObjectRequest :md5sum-) "937c9679a518e3a18d831e57125ea522")
(setf (get assignment2_package::PickObjectRequest :datatype-) "assignment2_package/PickObjectRequest")
(setf (get assignment2_package::PickObjectRequest :definition-)
      "# PickObject.srv
# (Empty request)
---
bool success
string message

")

(setf (get assignment2_package::PickObjectResponse :md5sum-) "937c9679a518e3a18d831e57125ea522")
(setf (get assignment2_package::PickObjectResponse :datatype-) "assignment2_package/PickObjectResponse")
(setf (get assignment2_package::PickObjectResponse :definition-)
      "# PickObject.srv
# (Empty request)
---
bool success
string message

")



(provide :assignment2_package/PickObject "937c9679a518e3a18d831e57125ea522")


