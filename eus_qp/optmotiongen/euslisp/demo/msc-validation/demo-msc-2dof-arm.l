(load "package://eus_qp/optmotiongen/euslisp/trajectory-configuration-task.l")
(load "package://eus_qp/optmotiongen/euslisp/sqp-msc-optimization.l")
(load "package://eus_qp/optmotiongen/euslisp/validate-jacobian.l")
(load "./util.l")
(load "./two-dof-manipulator.l")


(defun demo-msc-2dof-arm
    (&key
     (use-msc? t)
     (loop-num 30)
     (with-obstacle? t)
     (num-msc 4)
     (dispersion-scale 1e0)
     (dispersion-scale-decrease-ratio 0.8)
     (initial-noise-variance 1e-6)
     (log-name (if use-msc? "config-msc.csv" "config.csv"))
     (log-dir (ros::resolve-ros-path "package://eus_qp/optmotiongen/logs/demo-msc-2dof-arm/"))
     )
  ;; setup robot
  (setq *robot* (instance two-dof-manipulator :init))
  (send-all (send *robot* :joint-list) :max-angle 360)
  (send-all (send *robot* :joint-list) :min-angle -360)
  (unless with-obstacle?
    (send (elt (send *robot* :joint-list) 0) :max-angle 20)
    (send (elt (send *robot* :joint-list) 0) :min-angle -270)
    )
  (setq *robot-env*
        (instance robot-environment :init :robot *robot* :root-virtual-mode :fix))
  ;; setup environment
  (setq *obst1* (make-cube 500 500 500))
  (send *obst1* :newcoords (make-coords :pos (float-vector 500 500 0)))
  (cond (with-obstacle?
         (setq *obst-list* (list *obst1*))
         )
        (t
         (setq *obst-list* nil)
         ))
  (send-all *obst-list* :set-color (float-vector 0 1 1))
  ;; draw
  (unless (boundp '*irtviewer*)
    (make-irtviewer)
    (send *irtviewer* :change-background (float-vector 1 1 1))
    (send *irtviewer* :draw-floor 500)
    (send *irtviewer* :floor-color (float-vector 0 0 0))
    (send *irtviewer* :draw-origin nil)
    )
  (objects (append (list *robot*) *obst-list*))
  (send *irtviewer* :viewer :viewing :look #f(0 0 12000) #f(0 0 0) #f(1 0 0))
  (send *irtviewer* :draw-objects)

  (setq *instant-config-task*
        (instance instant-configuration-task :init
                  :robot-env *robot-env*
                  :only-kinematics? t
                  :drive-joint-list nil
                  :kin-target-coords-list
                  (list (make-coords :pos (float-vector 0 1000 0)))
                  :kin-attention-coords-list
                  (list (send *robot* :end-coords))
                  :kin-scale-list
                  (list (float-vector 1 1 0 0 0 0)) ;; only x and y
                  :collision-pair-list
                  (apply
                   #'append
                   (mapcar #'(lambda (l)
                               (mapcar #'(lambda (obst) (list l obst))
                                       *obst-list*))
                           (send *robot* :links))
                   )
                  :collision-distance-margin 0.01
                  :norm-regular-scale-max 1e10
                  ))

  (cond (use-msc?
         (setq *sqp-opt*
               (instance sqp-msc-optimization :init
                         :num-msc num-msc
                         :dispersion-scale dispersion-scale
                         :dispersion-scale-decrease-ratio dispersion-scale-decrease-ratio
                         :config-task *instant-config-task*
                         :pre-process-func
                         #'(lambda (sqp)
                             (send *irtviewer* :viewer :viewing :look #f(0 0 12000) #f(0 0 0) #f(1 0 0))
                             )
                         :post-process-func
                         #'(lambda (sqp)
                             (warning-message 3 "dispersion-scale: ~a~%"
                                              (send sqp :get-val '_dispersion-scale))
                             (write-configuration :sqp sqp)
                             )
                         ))
         (dolist (config-task (send *sqp-opt* :config-task-list))
           (send config-task :set-config
                 (v+ (send config-task :config-vector)
                     (gaussian-random 2 0 (sqrt initial-noise-variance))
                     ))
           )
         (objects (append (send-all (send *sqp-opt* :config-task-list) :robot-env) *obst-list*))
         (send *irtviewer* :viewer :viewing :look #f(0 0 12000) #f(0 0 0) #f(1 0 0))
         (send *irtviewer* :draw-objects)
         )
        (t
         (setq *sqp-opt*
               (instance sqp-optimization :init
                         :config-task *instant-config-task*
                         :pre-process-func
                         #'(lambda (sqp)
                             (send *irtviewer* :viewer :viewing :look #f(0 0 12000) #f(0 0 0) #f(1 0 0))
                             )
                         :post-process-func
                         #'(lambda (sqp)
                             ;; (validate-jacobian :config-task (send sqp :config-task))
                             (write-configuration :sqp sqp)
                             )
                         ))
         ))
  (send *sqp-opt* :put :filename (format nil "~a/~a" log-dir log-name))
  (unix:system
   (format nil "DIRNAME=`dirname ~a`; if [ ! -d $DIRNAME ]; then mkdir -p $DIRNAME; fi" (send *sqp-opt* :get :filename)))
  (write-configuration :sqp *sqp-opt* :initialize? t)
  (send *sqp-opt* :optimize
        :loop-num loop-num
        :print-status-interval 1
        )
  )
(warn "(demo-msc-2dof-arm)~%")

#|
;; joint range
(demo-msc-2dof-arm :use-msc? nil :with-obstacle? nil)
(demo-msc-2dof-arm :use-msc? t :with-obstacle? nil)
;; obstacle
(demo-msc-2dof-arm :use-msc? nil :with-obstacle? t)
(demo-msc-2dof-arm :use-msc? t :with-obstacle? t)
