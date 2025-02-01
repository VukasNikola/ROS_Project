
(cl:in-package :asdf)

(defsystem "assignment2_package-srv"
  :depends-on (:roslisp-msg-protocol :roslisp-utils :geometry_msgs-msg
)
  :components ((:file "_package")
    (:file "PickObject" :depends-on ("_package_PickObject"))
    (:file "_package_PickObject" :depends-on ("_package"))
    (:file "PlaceObject" :depends-on ("_package_PlaceObject"))
    (:file "_package_PlaceObject" :depends-on ("_package"))
  ))