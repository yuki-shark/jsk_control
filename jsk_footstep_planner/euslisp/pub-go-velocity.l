#!/usr/bin/env roseus
(require "package://hrpsys_ros_bridge_tutorials/euslisp/jaxon_red-interface.l")
(unless (boundp '*jaxon_red*)
  (jaxon_red-init)
  (setq *robot* *jaxon_red*)
  )
(ros::roseus-add-msgs "geometry_msgs")

(defun start-footguided-modification
    ()
  (let* ((step-time 0.7)
         (double-ratio 0.1))
  ;; (let* ((step-time 1.0)
  ;;        (double-ratio 0.1))
    (send *ri* :set-gait-generator-param
          :leg-margin #f(0.12 0.09 0.05 0.05)
          :safe-leg-margin #f(0.07 0.055 0.05 0.05)
          :dcm-offset 0.02
          )
    (send *ri* :set-gait-generator-param
          :stride-limitation-type 1
          :stride-limitation-for-circle-type #f(0.15 0.3 15 0.1 0.14)
          :default-orbit-type :rectangle
          ;; :default-step-height 0.05
          :default-step-height 0.07
          :default-step-time step-time
          :default-double-support-ratio double-ratio
          :swing-trajectory-delay-time-offset (* step-time (- 1.0 double-ratio) 0.25)
          ;; :overwritable-stride-limitation #f(0.35 0.5 0 0.3 0.14)
          :overwritable-stride-limitation #f(0.35 0.5 0 0.4 0.14)
          :margin-time-ratio 0.2
          :min-time-mgn 0.3
          :min-time 0.6
          ;; :min-time 0.8
          :use-disturbance-compensation t
          :zmp-delay-time-const 0.05
          :dc-gain 1e-4
          :emergency-step-time #f(0.08 0.6 0.7)
          )
    (send *ri* :set-auto-balancer-param
          :is-emergency-step-mode t
          :is-emergency-touch-wall-mode nil
          :touch-wall-retrieve-time 0.4
          )
    ))

(defun push-init
    (&optional (use-hand? nil))
  (when (boundp '*jaxon_red*)
    (send *ri* :stop-impedance :arms))
    ;; (send *ri* :start-grasp))
  (send *ri* :autobalancerservice_stopStabilizer)
  (send *ri* :stop-auto-balancer)
  (send *robot* :reset-pose)
  (send *robot* :fix-leg-to-coords (make-coords))
  (send *robot* :arms :elbow-p :joint-angle -40)
  (send *robot* :arms :shoulder-p :joint-angle 20)
  (send *robot* :move-centroid-on-foot :both '(:rleg :lleg))
  (send *ri* :angle-vector (send *robot* :angle-vector) 2000)
  (send *ri* :wait-interpolation)
  (send *ri* :set-auto-balancer-param
        :is-hand-fix-mode nil
        :use-limb-stretch-avoidance nil
        :ik-mode 1)
  (if use-hand?
      (send *ri* :start-auto-balancer)
    (send *ri* :start-auto-balancer :limbs '(:rleg :lleg)))
  (send *ri* :autobalancerservice_startStabilizer)
  )

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

;; footguided modification
(format t "start footguided modification~%")
(start-footguided-modification)
;; when using hand
;; (push-init t)

;; move arms backward
(format t "move arms backward~%")
(send *ri* :angle-vector #f(0.0 0.0 -20.0 40.0 -20.0 0.0 0.0 0.0 -20.0 40.0 -20.0 0.0 0.0 0.0 0.0 0.0 0.0 0.0 60.0 -20.0 -5.0 -80.0 0.0 0.0 -20.0 0.0 60.0 20.0 5.0 -80.0 0.0 0.0 -20.0) 3000)

(ros::roseus "pub_go_velocity")
(format t "pub_go_velocity node init~%")
(ros::subscribe "cmd_vel" geometry_msgs::Twist #'callback 1)
(do-until-key
 (ros::spin-once))
(format t "Shutdown go-velocity Publisher~%")
