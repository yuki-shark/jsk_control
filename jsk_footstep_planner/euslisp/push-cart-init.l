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
;; (read-line)
(progn
  (send *ri* :angle-vector
        #f(0.00762 0.016455 -41.4751 54.4828 -23.958 -0.055639 -0.012603 0.015461 -41.4724 54.4764 -23.9184 -0.061615 -0.035926 0.024968 3.018715e-05 0.000367 35.1754 32.9447 29.621 -42.6064 6.23004 -74.2283 -1.32756 -6.1922 -29.8771 -32.9393 29.6161 42.6033 -6.22909 -74.1809 1.32332 6.1922 -29.9017)
        5000))

;; grasp
(format t "grasp~%")
;; (read-line)
(send *ri* :wait-interpolation)
;; (send *ri* :emergency-release-motion)
(send *ri* :start-grasp)
