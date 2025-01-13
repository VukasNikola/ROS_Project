
(cl:in-package :asdf)

(defsystem "tiago_iaslab_simulation-srv"
  :depends-on (:roslisp-msg-protocol :roslisp-utils )
  :components ((:file "_package")
    (:file "Coeffs" :depends-on ("_package_Coeffs"))
    (:file "_package_Coeffs" :depends-on ("_package"))
    (:file "Objs" :depends-on ("_package_Objs"))
    (:file "_package_Objs" :depends-on ("_package"))
  ))