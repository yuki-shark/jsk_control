#!/usr/bin/env roseus
(require "package://hrpsys_ros_bridge_tutorials/euslisp/jaxon_red-interface.l")
(unless (boundp '*jaxon_red*)
  (jaxon_red-init)
  (setq *robot* *jaxon_red*)
  )

;; move arms backward
(format t "move arms backward~%")
(send *ri* :angle-vector #f(0.0 0.0 -20.0 40.0 -20.0 0.0 0.0 0.0 -20.0 40.0 -20.0 0.0 0.0 0.0 0.0 0.0 0.0 0.0 60.0 -20.0 -5.0 -80.0 0.0 0.0 -20.0 0.0 60.0 20.0 5.0 -80.0 0.0 0.0 -20.0) 3000)

(ros::roseus-add-msgs "geometry_msgs")

(defun callback(msg)
  (setq linear (send msg :linear))
  (setq angular (send msg :angular))

  (setq x (send linear :x))
  (setq y (send linear :y))
  (setq yaw (rad2deg (send angular :z)))

  ;; update command
  (send *ri* :go-velocity x y yaw)
  (format t "update command~%")

  ;; stop command
  (when (and (eq x 0.0) (eq y 0.0) (eq yaw 0.0))
    (send *ri* :go-stop)
    (format t "stop command~%")
    )
  ) ;; defun

(ros::roseus "pub_go_velocity")
(format t "pub_go_velocity node init~%")
(ros::subscribe "cmd_vel" geometry_msgs::Twist #'callback 1)
(do-until-key
 (ros::spin-once))
(format t "Shutdown go-velocity Publisher~%")
