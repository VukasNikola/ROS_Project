
(cl:in-package :asdf)

(defsystem "assignment2_package-srv"
  :depends-on (:roslisp-msg-protocol :roslisp-utils :geometry_msgs-msg
)
  :components ((:file "_package")
    (:file "GetObjectPose" :depends-on ("_package_GetObjectPose"))
    (:file "_package_GetObjectPose" :depends-on ("_package"))
    (:file "PickObject" :depends-on ("_package_PickObject"))
    (:file "_package_PickObject" :depends-on ("_package"))
  ))