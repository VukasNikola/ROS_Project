;; Auto-generated. Do not edit!


(when (boundp 'tiago_iaslab_simulation::Objs)
  (if (not (find-package "TIAGO_IASLAB_SIMULATION"))
    (make-package "TIAGO_IASLAB_SIMULATION"))
  (shadow 'Objs (find-package "TIAGO_IASLAB_SIMULATION")))
(unless (find-package "TIAGO_IASLAB_SIMULATION::OBJS")
  (make-package "TIAGO_IASLAB_SIMULATION::OBJS"))
(unless (find-package "TIAGO_IASLAB_SIMULATION::OBJSREQUEST")
  (make-package "TIAGO_IASLAB_SIMULATION::OBJSREQUEST"))
(unless (find-package "TIAGO_IASLAB_SIMULATION::OBJSRESPONSE")
  (make-package "TIAGO_IASLAB_SIMULATION::OBJSRESPONSE"))

(in-package "ROS")





(defclass tiago_iaslab_simulation::ObjsRequest
  :super ros::object
  :slots (_ready ))

(defmethod tiago_iaslab_simulation::ObjsRequest
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

(defclass tiago_iaslab_simulation::ObjsResponse
  :super ros::object
  :slots (_ids ))

(defmethod tiago_iaslab_simulation::ObjsResponse
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

(defclass tiago_iaslab_simulation::Objs
  :super ros::object
  :slots ())

(setf (get tiago_iaslab_simulation::Objs :md5sum-) "b0d667d823cb9909fa2a290df26f97da")
(setf (get tiago_iaslab_simulation::Objs :datatype-) "tiago_iaslab_simulation/Objs")
(setf (get tiago_iaslab_simulation::Objs :request) tiago_iaslab_simulation::ObjsRequest)
(setf (get tiago_iaslab_simulation::Objs :response) tiago_iaslab_simulation::ObjsResponse)

(defmethod tiago_iaslab_simulation::ObjsRequest
  (:response () (instance tiago_iaslab_simulation::ObjsResponse :init)))

(setf (get tiago_iaslab_simulation::ObjsRequest :md5sum-) "b0d667d823cb9909fa2a290df26f97da")
(setf (get tiago_iaslab_simulation::ObjsRequest :datatype-) "tiago_iaslab_simulation/ObjsRequest")
(setf (get tiago_iaslab_simulation::ObjsRequest :definition-)
      "bool ready          #if true apriltag_ids sends the apriltags IDs, otherwise prints an error and kill ros
---
int32[] ids

")

(setf (get tiago_iaslab_simulation::ObjsResponse :md5sum-) "b0d667d823cb9909fa2a290df26f97da")
(setf (get tiago_iaslab_simulation::ObjsResponse :datatype-) "tiago_iaslab_simulation/ObjsResponse")
(setf (get tiago_iaslab_simulation::ObjsResponse :definition-)
      "bool ready          #if true apriltag_ids sends the apriltags IDs, otherwise prints an error and kill ros
---
int32[] ids

")



(provide :tiago_iaslab_simulation/Objs "b0d667d823cb9909fa2a290df26f97da")


