(load "../inverse-kinematics-statics-wrapper.l")
(require "~/prog/rcb4eus/makabe.l")
(require "models/arrow-object.l")


(defun demo-kxr-inverse-kinematics-statics
    (&key
     (optimize-torque? nil)
     (pre-process-func)
     (post-process-func)
     &allow-other-keys
     )
  (setq *robot* (kxr-make-robot "kxrl2makabe"))
  (send *robot* :reset-pose)
  (send *robot* :fix-leg-to-coords (make-coords))
  (create-support-polygon *robot* (send (send *robot* :rleg :end-coords) :parent))
  (create-support-polygon *robot* (send (send *robot* :lleg :end-coords) :parent))
  (setq *robot-env*
        (instance robot-environment :init :robot *robot* :root-virtual-mode :6dof))
  (setq *arrow* (instance arrow-object :init))
  (send *arrow* :newcoords
        (make-coords :pos (float-vector 200 0 100) :rpy (list 0 0 0)))
  (unless (boundp '*irtviewer*)
    (make-irtviewer)
    (send *irtviewer* :change-background (float-vector 1 1 1))
    (send *irtviewer* :draw-floor 500)
    (send *irtviewer* :floor-color (float-vector 0 0 0))
    )
  (objects (list *robot* *arrow*))

  (setq *instant-config-task*
        (instance instant-configuration-task :init
                  :robot-env *robot-env*
                  :drive-joint-list (if optimize-torque? (send *robot-env* :drive-joint-list))
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
                             :init (float-vector 0 0 0 0 0 0))
                             ;; :init (float-vector 20 20 20 0 0 0))
                   )
                  :norm-regular-scale-max 1e-5
                  :norm-regular-scale-offset 1e-6
                  ))

  (setq *sqp-opt*
        (instance sqp-optimization :init
                  :config-task *instant-config-task*
                  :pre-process-func pre-process-func
                  :post-process-func post-process-func
                  ))
  (send *sqp-opt* :optimize :loop-num 50)
  t)
(warn "(demo-kxr-inverse-kinematics-statics)~%")
