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
;; (push-init t)

;; move arms backward
(format t "move arms backward~%")
(send *robot* :angle-vector #f(0.0 0.0 -20.0 40.0 -20.0 0.0 0.0 0.0 -20.0 40.0 -20.0 0.0 0.0 0.0 0.0 0.0 30.0 0.0 70.0 -45.0 -5.0 -80.0 0.0 0.0 -20.0 0.0 70.0 45.0 5.0 -80.0 0.0 0.0 -20.0))
(send *robot* :fix-leg-to-coords (make-coords))
(send *robot* :legs :move-end-pos #f(0 0 60))
(send *ri* :angle-vector (send *robot* :angle-vector) 3000)

;; hand grasp
(send *ri* :emergency-release-motion)
(send *ri* :start-grasp)
