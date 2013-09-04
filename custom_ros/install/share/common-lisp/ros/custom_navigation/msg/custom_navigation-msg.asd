
(cl:in-package :asdf)

(defsystem "custom_navigation-msg"
  :depends-on (:roslisp-msg-protocol :roslisp-utils :std_msgs-msg
)
  :components ((:file "_package")
    (:file "M_ADC" :depends-on ("_package_M_ADC"))
    (:file "_package_M_ADC" :depends-on ("_package"))
  ))