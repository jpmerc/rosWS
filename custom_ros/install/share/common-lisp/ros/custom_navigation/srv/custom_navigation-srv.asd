
(cl:in-package :asdf)

(defsystem "custom_navigation-srv"
  :depends-on (:roslisp-msg-protocol :roslisp-utils )
  :components ((:file "_package")
    (:file "S_ADC" :depends-on ("_package_S_ADC"))
    (:file "_package_S_ADC" :depends-on ("_package"))
    (:file "S_DigitalOut" :depends-on ("_package_S_DigitalOut"))
    (:file "_package_S_DigitalOut" :depends-on ("_package"))
  ))