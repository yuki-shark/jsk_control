#!/usr/bin/env roseus
(require "package://hrpsys_ros_bridge_tutorials/euslisp/jaxon_red-interface.l")
(unless (boundp '*jaxon_red*)
  (jaxon_red-init)
  (setq *robot* *jaxon_red*)
  )
(ros::roseus-add-msgs "geometry_msgs")
(ros::roseus-add-msgs "move_base_msgs")

(defun callback(msg)
  (setq linear (send msg :linear))
  (setq angular (send msg :angular))

  (setq x (send linear :x))
  (setq y (send linear :y))
  (setq yaw (rad2deg (send angular :z)))

  (when (eq *mode* 1)
    (if (> (abs (/ x 0.2)) (abs (/ (send angular :z) 0.128)))
        (setq yaw 0)
      (setq x 0))
    )

  ;; transform cart vel to robot vel
  ;; (setq l 1.1)
  (setq l *l*)
  (setq dt 0.7)
  (setq x (+ x (/ (- l (* l (cos (* (send angular :z) dt)))) dt)))
  (setq y (- y (/ (* l (sin (* (send angular :z) dt))) dt)))
  ;; (format t "l : ~A  ox : ~A  oy : ~A  yaw : ~A  dt : ~A  x : ~A  y: ~A~% " l (send linear :x) (send linear :y) (send angular :z) dt x y)

  ;; (setq y (- y (* l (send angular :z))))

  ;; update command
  (send *ri* :go-velocity x y yaw)
  (format t "update command~%")
  ) ;; callback

(defun result-callback(msg)
  ;; stop command
  (when (eq (send (send msg :status) :status) 3)
    (send *ri* :go-stop)
    (format t "stop command~%")
    )
  ) ;; result-callback

(ros::roseus "pub_go_velocity")
(format t "pub_go_velocity node init~%")
(setq *l* (ros::get-param "~distance_to_cart_center" 0.5))
(setq *mode* (ros::get-param "~forward_or_rotate_mode" 0))
(ros::subscribe "cmd_vel" geometry_msgs::Twist #'callback 1)
(ros::subscribe "move_base/result" move_base_msgs::MoveBaseActionResult #'result-callback 1)
(do-until-key
 (ros::spin-once))
(format t "Shutdown go-velocity Publisher~%")
