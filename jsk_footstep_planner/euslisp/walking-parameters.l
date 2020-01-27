#!/usr/bin/env roseus

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
         :default-step-height 0.14
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
         :sum-d-footstep-thre #f(0.3 0.3 0.0)
         :footstep-check-delta #f(0.02 0.02 0.0)
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
