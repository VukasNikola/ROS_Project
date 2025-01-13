
(cl:in-package :asdf)

(defsystem "apriltag_finder-srv"
  :depends-on (:roslisp-msg-protocol :roslisp-utils )
  :components ((:file "_package")
    (:file "ApriltagsIDsSrv" :depends-on ("_package_ApriltagsIDsSrv"))
    (:file "_package_ApriltagsIDsSrv" :depends-on ("_package"))
  ))