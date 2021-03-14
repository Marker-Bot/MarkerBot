
(cl:in-package :asdf)

(defsystem "hrwros_msgs-msg"
  :depends-on (:roslisp-msg-protocol :roslisp-utils )
  :components ((:file "_package")
    (:file "Points_arrays" :depends-on ("_package_Points_arrays"))
    (:file "_package_Points_arrays" :depends-on ("_package"))
  ))