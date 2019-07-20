(load "package://eus_qp/optmotiongen/euslisp/trajectory-configuration-task.l")
(load "package://eus_qp/optmotiongen/euslisp/inverse-kinematics-statics-wrapper.l")
(load "package://eus_qp/optmotiongen/euslisp/sqp-optimization.l")
(load "package://eus_qp/optmotiongen/euslisp/validate-jacobian.l")
(require :jaxon_red "package://hrpsys_ros_bridge_tutorials/euslisp/jaxon_red-interface.l")
(require "models/arrow-object.l")


(defun setup-robot-for-demo-jaxon-external-force
    ()
  ;; robot
  (setq *robot* (instance jaxon_red-robot :init))
  (send *robot* :reset-pose)
  (send *robot* :fix-leg-to-coords (make-coords))
  (send *robot* :put :reset-pose-joint-angle-list (coerce (send *robot* :angle-vector) cons))
  ;; contact constraint
  (create-support-polygon *robot* (send (send *robot* :lleg-end-coords) :parent))
  (send *robot* :put :lleg-foot-contact-constraint
        (instance* default-contact-constraint
                   :init
                   :mu-trans 0.2
                   :mu-rot 0.01
                   :max-fz 1000
                   (let* ((poly
                           (send (send (send *robot* :lleg-end-coords) :parent) :get :support-polygon))
                          (vs
                           (mapcar #'(lambda (v)
                                       (send (send (send *robot* :lleg-end-coords) :worldcoords)
                                             :inverse-transform-vector v))
                                   (send poly :vertices)
                                   ))
                          )
                     (list :l-min-x (+ (elt (find-extream vs #'(lambda (v) (elt v 0)) #'<) 0) 20)
                           :l-max-x (- (elt (find-extream vs #'(lambda (v) (elt v 0)) #'>) 0) 20)
                           :l-min-y (+ (elt (find-extream vs #'(lambda (v) (elt v 1)) #'<) 1) 20)
                           :l-max-y (- (elt (find-extream vs #'(lambda (v) (elt v 1)) #'>) 1) 20)
                           ))
                   ))
  (create-support-polygon *robot* (send (send *robot* :rleg-end-coords) :parent))
  (send *robot* :put :rleg-foot-contact-constraint
        (instance* default-contact-constraint
                   :init
                   :mu-trans 0.2
                   :mu-rot 0.01
                   :max-fz 1000
                   (let* ((poly
                           (send (send (send *robot* :rleg-end-coords) :parent) :get :support-polygon))
                          (vs
                           (mapcar #'(lambda (v)
                                       (send (send (send *robot* :rleg-end-coords) :worldcoords)
                                             :inverse-transform-vector v))
                                   (send poly :vertices)
                                   ))
                          )
                     (list :l-min-x (+ (elt (find-extream vs #'(lambda (v) (elt v 0)) #'<) 0) 20)
                           :l-max-x (- (elt (find-extream vs #'(lambda (v) (elt v 0)) #'>) 0) 20)
                           :l-min-y (+ (elt (find-extream vs #'(lambda (v) (elt v 1)) #'<) 1) 20)
                           :l-max-y (- (elt (find-extream vs #'(lambda (v) (elt v 1)) #'>) 1) 20)
                           ))
                   ))
  )

(defun demo-jaxon-external-force
    (&key
     (larm-hand-fz 200)
     (optimize-torque? nil)
     (pre-process-func)
     &allow-other-keys
     )
  ;; setup
  (setup-robot-for-demo-jaxon-external-force)
  (setq *robot-env*
        (instance robot-environment :init :robot *robot*))
  (setq *arrow* (instance arrow-object :init))
  (send *arrow* :newcoords (make-coords :pos (float-vector 700 300 1000)))
  ;; draw
  (unless (boundp '*irtviewer*)
    (make-irtviewer)
    (send *irtviewer* :change-background (float-vector 1 1 1))
    (send *irtviewer* :draw-floor 500)
    (send *irtviewer* :floor-color (float-vector 0 0 0))
    )
  (objects (list *robot* *arrow*))

  (setq *instant-config-task*
        (instance instant-configuration-task :init
                  :name :knee-stand
                  :robot-env *robot-env*
                  :drive-joint-list nil
                  :kin-target-coords-list
                  (list
                   (send (send *robot-env* :robot :lleg-end-coords) :copy-worldcoords)
                   (send (send *robot-env* :robot :rleg-end-coords) :copy-worldcoords)
                   (send *arrow* :copy-worldcoords)
                   )
                  :kin-attention-coords-list
                  (list
                   (send *robot-env* :robot :lleg-end-coords)
                   (send *robot-env* :robot :rleg-end-coords)
                   (send *robot-env* :robot :larm-end-coords)
                   )
                  :contact-target-coords-list
                  (list
                   (send (send *robot-env* :robot :lleg-end-coords) :copy-worldcoords)
                   (send (send *robot-env* :robot :rleg-end-coords) :copy-worldcoords)
                   )
                  :contact-attention-coords-list
                  (list
                   (send *robot-env* :robot :lleg-end-coords)
                   (send *robot-env* :robot :rleg-end-coords)
                   )
                  :contact-constraint-list
                  (list
                   (send *robot-env* :robot :get :lleg-foot-contact-constraint)
                   (send *robot-env* :robot :get :rleg-foot-contact-constraint)
                   )
                  :external-wrench-list
                  (list
                   (float-vector 0 0 larm-hand-fz 0 0 0)
                   )
                  :external-coords-list
                  (list
                   (send *robot-env* :robot :larm-end-coords)
                   )
                  :collision-pair-list
                  (mapcar #'(lambda (l) (list l (send *robot* :link "CHEST_LINK1"))) (subseq (send *robot* :larm) 4))
                  :collision-distance-margin
                  0.02
                  :posture-joint-list
                  (send *robot-env* :robot :joint-list)
                  :posture-joint-angle-list
                  (send *robot-env* :robot :get :reset-pose-joint-angle-list)
                  :target-posture-scale 1e-3
                  ))
  (send *instant-config-task* :put :thre (make-list 3 :initial-element 1)) ;; [mm]
  (send *instant-config-task* :put :rthre (make-list 3 :initial-element (deg2rad 1))) ;; [rad]
  (send *instant-config-task* :put :eom-trans-thre 1e-2)
  (send *instant-config-task* :put :eom-rot-thre 1e-2)
  (send *instant-config-task* :put :torque-thre 1e-2)

  (setq *sqp-opt*
        (instance sqp-optimization :init
                  :config-task *instant-config-task*
                  :convergence-check-func #'sqp-convergence-check
                  :failure-callback-func #'sqp-failure-callback
                  ))
  (send *sqp-opt* :optimize :loop-num 50)
  )
(warn "(demo-jaxon-external-force)~%")
(warn "(demo-jaxon-external-force :larm-hand-fz -100)~%")

(defun loop-demo-jaxon-external-force
    ()
  (demo-jaxon-external-force)
  (let* ((i 0)
         (fz)
         )
    (do-until-key
     (incf i)
     (setq fz (* 400 (sin (* 0.01 (float i)))))
     (send *instant-config-task* :external-wrench-list (list (float-vector 0 0 fz 0 0 0)))
     (send *sqp-opt* :optimize
           :update-viewer-interval nil
           :print-status-interval nil
           :loop-num 50
           :loop-num-min 2
           )
     ))
  )
(warn "(loop-demo-jaxon-external-force)~%")
