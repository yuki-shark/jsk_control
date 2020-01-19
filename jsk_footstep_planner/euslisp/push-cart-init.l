#!/usr/bin/env roseus
(require "package://hrpsys_ros_bridge_tutorials/euslisp/jaxon_red-interface.l")
(unless (boundp '*jaxon_red*)
  (jaxon_red-init)
  (setq *robot* *jaxon_red*)
  )

;; choreonoid terrain env param
(defun start-footguided-modification
   ()
 (let* ((step-time 0.7)
        (double-ratio 0.05))
   (cond ((boundp '*chidori*)
          (send *ri* :set-gait-generator-param
                :leg-margin #f(0.15 0.09 0.065 0.065)
                :safe-leg-margin #f(0.13 0.07 0.05 0.05)
                ))
         ((boundp '*jaxon_red*)
          (send *ri* :set-gait-generator-param
                :leg-margin #f(0.12 0.09 0.05 0.05)
                :safe-leg-margin #f(0.07 0.055 0.03 0.03)
                :dcm-offset 0.0
                :sum-d-footstep-thre #f(0.1 0.1 0)
                ))
         ((boundp '*rhp4b*)
          (send *ri* :set-gait-generator-param
                :leg-margin #f(0.12 0.09 0.07 0.06)
                :safe-leg-margin #f(0.07 0.055 0.07 0.06)
                :dcm-offset 0.02
                ))
         )
   (send *ri* :set-gait-generator-param
         :stride-limitation-type 1
         ;; :stride-limitation-for-circle-type #f(0.15 0.3 15 0.1 0.14)
         :stride-limitation-for-circle-type #f(0.15 0.4 15 0.1 0.14) ;; [normal] front outside angle rear inside
         ;; max vel_x = front / step-time
         ;; max vel_y = (outside - inside) / (2 * step-time)
         :default-orbit-type :rectangle
         :default-step-height 0.1
         :default-step-time step-time
         :default-double-support-ratio double-ratio
         :swing-trajectory-delay-time-offset (* step-time (- 1.0 double-ratio) 0.25)
         :overwritable-stride-limitation #f(0.35 0.55 0 0.35 0.14) ;; [footstep modification] front outside angle rear inside
         :margin-time-ratio 0.1
         :min-time-mgn 0.3
         :min-time 0.6
         :use-disturbance-compensation t
         :zmp-delay-time-const 0.05
         :dc-gain 5e-4
         :emergency-step-time #f(0.08 0.6 0.7)
         :overwritable-max-time 1.0
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
        :is-hand-fix-mode t
        :use-limb-stretch-avoidance nil
        :ik-mode 1)
  (if use-hand?
      (send *ri* :start-auto-balancer)
    (send *ri* :start-auto-balancer :limbs '(:rleg :lleg)))
  (send *ri* :autobalancerservice_startStabilizer)
  )

;; footguided modification
(format t "start footguided modification~%")
(start-footguided-modification)
(format t "push init~%")
(push-init t)

;; move to pregrasp pos
(format t "move to pregrasp pos~%")
(progn
  (send *ri* :start-impedance :arms)
  (send *ri* :angle-vector
        #f(0.00762 0.016455 -41.4751 54.4828 -23.958 -0.055639 -0.012603 0.015461 -41.4724 54.4764 -23.9184 -0.061615 -0.035926 0.024968 3.018715e-05 0.000367 35.1754 20.9694 41.5013 -52.4274 -3.06923 -72.1906 -2.19003 1.88788 -46.9356 -20.9567 41.5045 52.4412 3.0781 -72.1778 2.19183 -1.89501 -46.9354)
        5000))
(send *ri* :wait-interpolation)

;; move to grasp pos
(format t "move to grasp pos~%")
(progn
  (send *ri* :angle-vector
        #f(0.00762 0.016455 -41.4751 54.4828 -23.958 -0.055639 -0.012603 0.015461 -41.4724 54.4764 -23.9184 -0.061615 -0.035926 0.024968 3.018715e-05 0.000367 35.1754 32.9447 29.621 -42.6064 6.23004 -74.2283 -1.32756 -6.1922 -29.8771 -32.9393 29.6161 42.6033 -6.22909 -74.1809 1.32332 6.1922 -29.9017)
        5000))

;; grasp
(format t "grasp~%")
(send *ri* :wait-interpolation)
;; (send *ri* :emergency-release-motion)
(send *ri* :start-grasp)
