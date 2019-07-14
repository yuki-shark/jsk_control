(load "../instant-configuration-task.l")
(load "../compound-configuration-task.l")
(load "../sqp-optimization.l")
(load "../validate-jacobian.l")
(require "irteus/demo/sample-arm-model.l")
(require "irteus/demo/sample-robot-model.l")
(require "models/arrow-object.l")


(defun sample-sqp-optimization-compound
    (&key
     (avoid-collision? t)
     (loop-num 50)
     (pre-process-func)
     &allow-other-keys
     )
  ;; setup robot
  (setq *robot* (instance sample-robot :init))
  (send *robot* :reset-pose)
  (send *robot* :fix-leg-to-coords (make-coords))
  (create-support-polygon *robot* (send (send *robot* :rleg :end-coords) :parent))
  (create-support-polygon *robot* (send (send *robot* :lleg :end-coords) :parent))
  (setq *robot2* (instance sarmclass :init))
  (send *robot2* :newcoords (make-coords :pos (float-vector -500 0 0)))
  (send *robot2* :init-pose)
  ;; setup environment
  (setq *arrow* (instance arrow-object :init))
  (send *arrow* :newcoords
        (make-coords :pos (float-vector 400 -200 600) :rpy (list 0 0 0)))
  (setq *arrow2* (instance arrow-object :init))
  (send *arrow2* :newcoords
        (make-coords :pos (float-vector 400 -200 300) :rpy (list 0 (deg2rad -30) 0)))
  ;; setup contact
  (setq *robot-env*
        (instance robot-environment :init :robot *robot* :root-virtual-mode :6dof))
  (setq *robot-env2*
        (instance robot-environment :init :robot *robot2* :root-virtual-mode :planar))
  ;; draw
  (unless (boundp '*irtviewer*)
    (make-irtviewer)
    (send *irtviewer* :change-background (float-vector 1 1 1))
    (send *irtviewer* :draw-floor 500)
    (send *irtviewer* :floor-color (float-vector 0 0 0))
    )
  (objects (list *robot* *robot2* *arrow* *arrow2*))

  (setq *instant-config-task*
        (instance instant-configuration-task :init
                  :robot-env *robot-env*
                  :drive-joint-list nil
                  :kin-target-coords-list
                  (list (send (send *robot* :rleg :end-coords) :copy-worldcoords)
                        (send (send *robot* :lleg :end-coords) :copy-worldcoords)
                        (send *arrow* :copy-worldcoords)
                        )
                  :kin-attention-coords-list
                  (list (send *robot* :rleg :end-coords)
                        (send *robot* :lleg :end-coords)
                        (send *robot* :rarm :end-coords)
                        )
                  :contact-target-coords-list
                  (list (send (send *robot* :rleg :end-coords) :copy-worldcoords)
                        (send (send *robot* :lleg :end-coords) :copy-worldcoords)
                        (send *arrow* :copy-worldcoords)
                        )
                  :contact-attention-coords-list
                  (list (send *robot* :rleg :end-coords)
                        (send *robot* :lleg :end-coords)
                        (send *robot* :rarm :end-coords)
                        )
                  :contact-constraint-list
                  (list
                   (instance default-contact-constraint
                             :init-from-support-polygon
                             :support-polygon (send (send (send *robot-env* :robot :rleg :end-coords) :parent) :get :support-polygon)
                             :support-polygon-coords (send (send *robot-env* :robot :rleg :end-coords) :worldcoords)
                             )
                   (instance default-contact-constraint
                             :init-from-support-polygon
                             :support-polygon (send (send (send *robot-env* :robot :lleg :end-coords) :parent) :get :support-polygon)
                             :support-polygon-coords (send (send *robot-env* :robot :lleg :end-coords) :worldcoords)
                             )
                   (instance 6d-min-max-contact-constraint
                             :init (float-vector 20 20 20 0 0 0))
                   )
                  :collision-pair-list
                  (if avoid-collision?
                      (apply
                       #'append
                       (mapcar
                        #'(lambda (l1)
                            (mapcar
                             #'(lambda (l2)
                                 (list l1 l2))
                             (send *robot-env* :robot :links)))
                        (send *robot-env2* :robot :links))))
                  :norm-regular-scale-max 1e-5
                  :norm-regular-scale-offset 1e-6
                  ))

  (setq *instant-config-task2*
        (instance instant-configuration-task :init
                  :robot-env *robot-env2*
                  :drive-joint-list nil
                  :only-kinematics? t
                  :kin-target-coords-list
                  (list (send *arrow2* :copy-worldcoords))
                  :kin-attention-coords-list
                  (list (send *robot-env2* :robot :end-coords))
                  :collision-pair-list
                  (if avoid-collision?
                      (apply
                       #'append
                       (mapcar
                        #'(lambda (l1)
                            (mapcar
                             #'(lambda (l2)
                                 (list l1 l2))
                             (send *robot-env* :robot :links)))
                        (send *robot-env2* :robot :links))))
                  ))

  (setq *compound-config-task*
        (instance compound-configuration-task :init
                  :config-task-list (list *instant-config-task* *instant-config-task2*)
                  ))

  (setq *sqp-opt*
        (instance sqp-optimization :init
                  :config-task *compound-config-task*
                  :pre-process-func
                  pre-process-func
                  :post-process-func
                  #'(lambda (sqp)
                      (when (= (mod (send sqp :iteration) 20) 0)
                        ;; (validate-jacobian :config-task (send sqp :config-task))
                        ))
                  ))
  (send *sqp-opt* :optimize :loop-num loop-num)
  )
(warn "(sample-sqp-optimization-compound)~%")
