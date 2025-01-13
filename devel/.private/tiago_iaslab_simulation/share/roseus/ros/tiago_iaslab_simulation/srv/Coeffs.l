;; Auto-generated. Do not edit!


(when (boundp 'tiago_iaslab_simulation::Coeffs)
  (if (not (find-package "TIAGO_IASLAB_SIMULATION"))
    (make-package "TIAGO_IASLAB_SIMULATION"))
  (shadow 'Coeffs (find-package "TIAGO_IASLAB_SIMULATION")))
(unless (find-package "TIAGO_IASLAB_SIMULATION::COEFFS")
  (make-package "TIAGO_IASLAB_SIMULATION::COEFFS"))
(unless (find-package "TIAGO_IASLAB_SIMULATION::COEFFSREQUEST")
  (make-package "TIAGO_IASLAB_SIMULATION::COEFFSREQUEST"))
(unless (find-package "TIAGO_IASLAB_SIMULATION::COEFFSRESPONSE")
  (make-package "TIAGO_IASLAB_SIMULATION::COEFFSRESPONSE"))

(in-package "ROS")





(defclass tiago_iaslab_simulation::CoeffsRequest
  :super ros::object
  :slots (_ready ))

(defmethod tiago_iaslab_simulation::CoeffsRequest
  (:init
   (&key
    ((:ready __ready) nil)
    )
   (send-super :init)
   (setq _ready __ready)
   self)
  (:ready
   (&optional (__ready :null))
   (if (not (eq __ready :null)) (setq _ready __ready)) _ready)
  (:serialization-length
   ()
   (+
    ;; bool _ready
    1
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; bool _ready
       (if _ready (write-byte -1 s) (write-byte 0 s))
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; bool _ready
     (setq _ready (not (= 0 (sys::peek buf ptr- :char)))) (incf ptr- 1)
   ;;
   self)
  )

(defclass tiago_iaslab_simulation::CoeffsResponse
  :super ros::object
  :slots (_coeffs ))

(defmethod tiago_iaslab_simulation::CoeffsResponse
  (:init
   (&key
    ((:coeffs __coeffs) (make-array 0 :initial-element 0.0 :element-type :float))
    )
   (send-super :init)
   (setq _coeffs __coeffs)
   self)
  (:coeffs
   (&optional __coeffs)
   (if __coeffs (setq _coeffs __coeffs)) _coeffs)
  (:serialization-length
   ()
   (+
    ;; float32[] _coeffs
    (* 4    (length _coeffs)) 4
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; float32[] _coeffs
     (write-long (length _coeffs) s)
     (dotimes (i (length _coeffs))
       (sys::poke (elt _coeffs i) (send s :buffer) (send s :count) :float) (incf (stream-count s) 4)
       )
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; float32[] _coeffs
   (let (n)
     (setq n (sys::peek buf ptr- :integer)) (incf ptr- 4)
     (setq _coeffs (instantiate float-vector n))
     (dotimes (i n)
     (setf (elt _coeffs i) (sys::peek buf ptr- :float)) (incf ptr- 4)
     ))
   ;;
   self)
  )

(defclass tiago_iaslab_simulation::Coeffs
  :super ros::object
  :slots ())

(setf (get tiago_iaslab_simulation::Coeffs :md5sum-) "970e0019261c40a0b3c0c70ce65f2917")
(setf (get tiago_iaslab_simulation::Coeffs :datatype-) "tiago_iaslab_simulation/Coeffs")
(setf (get tiago_iaslab_simulation::Coeffs :request) tiago_iaslab_simulation::CoeffsRequest)
(setf (get tiago_iaslab_simulation::Coeffs :response) tiago_iaslab_simulation::CoeffsResponse)

(defmethod tiago_iaslab_simulation::CoeffsRequest
  (:response () (instance tiago_iaslab_simulation::CoeffsResponse :init)))

(setf (get tiago_iaslab_simulation::CoeffsRequest :md5sum-) "970e0019261c40a0b3c0c70ce65f2917")
(setf (get tiago_iaslab_simulation::CoeffsRequest :datatype-) "tiago_iaslab_simulation/CoeffsRequest")
(setf (get tiago_iaslab_simulation::CoeffsRequest :definition-)
      "bool ready          #if true get_straaightline_node sends the coefficients' values, otherwise prints an error and kill ros
---
float32[] coeffs

")

(setf (get tiago_iaslab_simulation::CoeffsResponse :md5sum-) "970e0019261c40a0b3c0c70ce65f2917")
(setf (get tiago_iaslab_simulation::CoeffsResponse :datatype-) "tiago_iaslab_simulation/CoeffsResponse")
(setf (get tiago_iaslab_simulation::CoeffsResponse :definition-)
      "bool ready          #if true get_straaightline_node sends the coefficients' values, otherwise prints an error and kill ros
---
float32[] coeffs

")



(provide :tiago_iaslab_simulation/Coeffs "970e0019261c40a0b3c0c70ce65f2917")


