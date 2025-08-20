
(cl:in-package :asdf)

(defsystem "assignment2_package-msg"
  :depends-on (:roslisp-msg-protocol :roslisp-utils :geometry_msgs-msg
)
  :components ((:file "_package")
    (:file "ObjectPose" :depends-on ("_package_ObjectPose"))
    (:file "_package_ObjectPose" :depends-on ("_package"))
    (:file "ObjectPoseArray" :depends-on ("_package_ObjectPoseArray"))
    (:file "_package_ObjectPoseArray" :depends-on ("_package"))
  ))