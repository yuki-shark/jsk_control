(load "package://eus_qp/optmotiongen/euslisp/trajectory-configuration-task.l")
(load "package://eus_qp/optmotiongen/euslisp/sqp-msc-optimization.l")
(load "package://eus_qp/optmotiongen/euslisp/validate-jacobian.l")
(load "./util.l")


(defun demo-msc-obstacle-2d
    (&key
     (use-msc? t)
     (loop-num 10)
     (L-shape-obst? t)
     (pos-max 5e3)
     (num-msc 4)
     (dispersion-scale 1e-1)
     (dispersion-scale-decrease-ratio 0.5)
     (initial-noise-variance 1e-6)
     (log-name (if use-msc? "config-msc.csv" "config.csv"))
     (log-dir (ros::resolve-ros-path "package://eus_qp/optmotiongen/logs/demo-msc-obstacle-2d/"))
     )
  ;; setup robot
  (let* ((body1 (make-cylinder 30 50))
         (clink (instance cascaded-link :init))
         )
    (send body1 :set-color (float-vector 1 0 1))
    (setq link1 (instance bodyset-link :init (make-cascoords) :bodies (list body1) :name :suitcase-link))
    (send clink :assoc link1)
    (setf (slot clink cascaded-link 'links) (list link1))
    (setf (slot clink cascaded-link 'joint-list) nil)
    (send clink :init-ending)
    (setq *robot* clink)
    (send *robot* :put :robot-coords
          (make-cascoords :parent (car (send *robot* :links))))
    )
  (setq *robot-env*
        (instance robot-environment :init :robot *robot*
                  :root-virtual-mode nil
                  :root-virtual-joint-class-list (list linear-joint linear-joint)
                  :root-virtual-joint-axis-list (list :x :y)
                  :root-virtual-joint-min-angle-list (list (- pos-max) (- pos-max))
                  :root-virtual-joint-max-angle-list (list pos-max pos-max)
                  ))
  (send *robot-env* :angle-vector (float-vector 2000 0))
  ;; setup environment
  (setq *obst1* (make-cube 200 2200 500))
  (send *obst1* :newcoords (make-coords :pos (float-vector 1000 0 0)))
  (cond (L-shape-obst?
         (setq *obst2* (make-cube 200 2200 500))
         (send *obst2* :newcoords (make-coords :pos (float-vector 0 1000 0) :rpy (list pi/2 0 0)))
         )
        (t
         (setq *obst2* nil)
         ))
  (setq *obst-list* (remove nil (list *obst1* *obst2*)))
  (send-all *obst-list* :set-color (float-vector 0 1 1))
  ;; draw
  (unless (boundp '*irtviewer*)
    (make-irtviewer)
    (send *irtviewer* :change-background (float-vector 1 1 1))
    (send *irtviewer* :draw-floor 500)
    (send *irtviewer* :floor-color (float-vector 0 0 0))
    )
  (objects (append (list *robot*) *obst-list*))

  (setq *instant-config-task*
        (instance instant-configuration-task :init
                  :robot-env *robot-env*
                  :only-kinematics? t
                  :kin-target-coords-list
                  (list (make-coords))
                  :kin-attention-coords-list
                  (list (send *robot* :get :robot-coords))
                  :kin-scale-list
                  (list (float-vector 1 1 0 0 0 0)) ;; only x and y
                  :collision-pair-list
                  (mapcar #'(lambda (obst) (list (car (send *robot* :links)) obst)) *obst-list*)
                  :collision-distance-margin
                  (- (* 1e-3 (radius-of-cylinder (car (send *robot* :bodies)))))
                  ))

  (cond (use-msc?
         (setq *sqp-opt*
               (instance sqp-msc-optimization :init
                         :num-msc num-msc
                         :dispersion-scale dispersion-scale
                         :dispersion-scale-decrease-ratio dispersion-scale-decrease-ratio
                         :config-task *instant-config-task*
                         :post-process-func
                         #'(lambda (sqp)
                             (send *irtviewer* :viewer :viewing :look #f(500 0 10000) #f(500 0 0) #f(1 0 0))
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
         )
        (t
         (setq *sqp-opt*
               (instance sqp-optimization :init
                         :config-task *instant-config-task*
                         :post-process-func
                         #'(lambda (sqp)
                             (send *irtviewer* :viewer :viewing :look #f(500 0 10000) #f(500 0 0) #f(1 0 0))
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
(warn "(demo-msc-obstacle-2d)~%")
