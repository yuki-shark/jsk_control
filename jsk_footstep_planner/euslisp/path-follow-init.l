#!/usr/bin/env roseus
(require "package://hrpsys_ros_bridge_tutorials/euslisp/jaxon_red-interface.l")
(require "package://jsk_footstep_planner/euslisp/walking-parameters.l")
(unless (boundp '*jaxon_red*)
  (jaxon_red-init)
  (setq *robot* *jaxon_red*)
  )

;; footguided modification
(format t "start footguided modification~%")
(start-footguided-modification)
;; when using hand
(push-init)

;; move arms backward
(format t "move arms backward~%")
;; (send *robot* :angle-vector #f(0.0 0.0 -20.0 40.0 -20.0 0.0 0.0 0.0 -20.0 40.0 -20.0 0.0 0.0 0.0 0.0 0.0 30.0 0.0 70.0 -45.0 -5.0 -80.0 0.0 0.0 -20.0 0.0 70.0 45.0 5.0 -80.0 0.0 0.0 -20.0))
;; (send *robot* :angle-vector #f(0.0 0.0 -20.0 40.0 -20.0 0.0 0.0 0.0 -20.0 40.0 -20.0 0.0 0.0 0.0 0.0 0.0 30.0 0.0 40.0 -20.0 -5.0 -80.0 0.0 0.0 -20.0 0.0 40.0 20.0 5.0 -80.0 0.0 0.0 -20.0))
;; (send *robot* :angle-vector #f(9.703657e-09 -4.528055e-07 -29.0846 58.1691 -29.0846 4.848231e-07 -9.703657e-09 4.528055e-07 -29.0846 58.1691 -29.0846 -4.848231e-07 0.0 0.0 0.0 0.0 0.0 0.0 40.0 -20.0 -5.0 -80.0 0.0 0.0 -20.0 0.0 40.0 20.0 5.0 -80.0 0.0 0.0 -20.0))
;; ;; (send *robot* :fix-leg-to-coords (make-coords))
;; ;; (send *robot* :legs :move-end-pos #f(0 0 60))
;; (send *ri* :angle-vector (send *robot* :angle-vector) 3000)
(send *robot* :reset-pose)
(send *robot* :fix-leg-to-coords (make-coords))
(send *robot* :arms :elbow-p :joint-angle -40)
(send *robot* :arms :shoulder-p :joint-angle 20)
(send *robot* :move-centroid-on-foot :both '(:rleg :lleg))
(send *robot* :legs :move-end-pos #f(0 0 50))
(send *robot* :larm :shoulder-r :joint-angle 40)
(send *robot* :rarm :shoulder-r :joint-angle -40)
(send *ri* :angle-vector (send *robot* :angle-vector) 2000)




;; hand grasp
(send *ri* :emergency-release-motion)
(send *ri* :start-grasp)
