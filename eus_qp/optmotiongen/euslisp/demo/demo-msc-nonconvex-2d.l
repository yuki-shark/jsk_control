(load "package://eus_qp/optmotiongen/euslisp/sqp-msc-optimization.l")


;;;;;;;;;;;;;;;;;;;;;;;;;
;; simple-2d-config-task
;;;;;;;;;;;;;;;;;;;;;;;;;

(defclass simple-2d-config-task
  :super propertied-object
  :slots (_config
          _dim-config
          _norm-regular-scale-max
          _norm-regular-scale-offset
          _norm-regular-scale-coeff
          _task-func
          _task-jacobi
          )
  )

(defmethod simple-2d-config-task
  (:init
   (&key
    (initial-config (float-vector 5.0 5.0))
    (norm-regular-scale-coeff 1.0)
    (norm-regular-scale-max 1e0)
    (norm-regular-scale-offset 1e-10)
    (config-min (float-vector 0 0))
    (config-max (float-vector 10 10))
    )
   (setq _config initial-config)
   (setq _dim-config 2)
   (setq _norm-regular-scale-coeff norm-regular-scale-coeff)
   (setq _norm-regular-scale-max norm-regular-scale-max)
   (setq _norm-regular-scale-offset norm-regular-scale-offset)
   (setq _config-min config-min)
   (setq _config-max config-max)
   )
  (:dim-config
   ()
   _dim-config
   )
  (:config-vector
   ()
   _config
   )
  (:set-config
   (config-new
    &key
    (relative? nil)
    )
   (cond (relative?
          (setq _config (v+ _config config-new))
          )
         (t
          (setq _config config-new)
          ))
   )
  (:task-value
   (&key
    (update?)
    &aux
    (x1 (elt _config 0))
    (x2 (elt _config 1))
    )
   (float-vector (funcall _task-func x1 x2))
   )
  (:task-jacobian
   (&aux
    (x1 (elt _config 0))
    (x2 (elt _config 1))
    )
   (matrix _task-jacobi)
   )
  (:config-inequality-constraint-matrix
   (&key
    (update? nil)
    )
   (when (or update? (not (send self :get :config-inequality-constraint-matrix)))
     (send self :put :config-inequality-constraint-matrix
           (concatenate-matrix-column
            (unit-matrix _dim-config)
            (scale-matrix -1 (unit-matrix _dim-config))
            )))
   (send self :get :config-inequality-constraint-matrix)
   )
  (:config-inequality-constraint-vector
   (&key
    (update? t)
    )
   (when (or update? (not (send self :get :config-inequality-constraint-vector)))
     (send self :put :config-inequality-constraint-vector
           (concatenate
            float-vector
            (v- _config-min _config)
            (v- _config _config-max)
            )))
   (send self :get :config-inequality-constraint-vector)
   )
  (:config-equality-constraint-matrix
   ()
   (make-matrix 0 _dim-config)
   )
  (:config-equality-constraint-vector
   ()
   (float-vector)
   )
  (:_dim-config-unit-matrix
   ()
   (or (send self :get :_dim-config-unit-matrix)
       (send self :put :_dim-config-unit-matrix (unit-matrix _dim-config)))
   )
  (:_dim-config-buf-matrix
   ()
   (or (send self :get :_dim-config-buf-matrix)
       (send self :put :_dim-config-buf-matrix (make-matrix _dim-config _dim-config)))
   )
  (:regular-matrix
   ()
   (scale-matrix
    (min _norm-regular-scale-max
         (+ (* _norm-regular-scale-coeff (norm2 (send self :task-value)))
            _norm-regular-scale-offset))
    (send self :_dim-config-unit-matrix)
    (send self :_dim-config-buf-matrix))
   )
  (:regular-vector
   ()
   (float-vector 0 0)
   )
  (:update-viewer
   ()
   )
  (:print-status
   ()
   (warning-message 9 "|task|=~a~%" (norm (send self :task-value :update? t)))
   (warning-message 9 "  config=~a~%" _config)
   )
  )


;;;;;;;;;;;;;;;;;;;;;;;;;
;; alpine2-config-task
;;;;;;;;;;;;;;;;;;;;;;;;;

(defun alpine2
    (x1 x2)
  (+ (* (sqrt x1) (sin x1) (sqrt x2) (sin x2)))
  )

(defun alpine2-jacobi
    (x1 x2)
  (float-vector
   (+ (* (/ 1.0 (* 2 (sqrt x1))) (sin x1) (sqrt x2) (sin x2))
      (* (sqrt x1) (cos x1) (sqrt x2) (sin x2)))
   (+ (* (sqrt x1) (sin x1) (/ 1.0 (* 2 (sqrt x2))) (sin x2))
      (* (sqrt x1) (sin x1) (sqrt x2) (cos x2)))
   )
  )

(defclass alpine2-config-task
  :super simple-2d-config-task
  )

(defmethod alpine2-config-task
  (:task-value
   (&key
    (update?)
    )
   (float-vector
    (- 8.0 (alpine2 (elt _config 0) (elt _config 1))))
   )
  (:task-jacobian
   ()
   (matrix
    (scale
     -1
     (alpine2-jacobi (elt _config 0) (elt _config 1))
     ))
   )
  )

(defun write-configuration
    (&key
     (sqp *sqp-opt*)
     (initialize? nil)
     &aux
     (config-list
      (cond ((derivedp sqp sqp-msc-optimization)
             (apply #'concatenate cons (send-all (send sqp :config-task-list) :config-vector))
             )
            ((derivedp sqp sqp-optimization)
             (concatenate cons (send (send sqp :config-task) :config-vector))
             ))
      )
     )
  (with-open-file
   (f (send sqp :get :filename)
      :direction :output :if-exists (if initialize? :new-version :append) :if-does-not-exist :create)
   (dolist (v config-list)
     (format f "~a, " v)
     )
   (format f "~%")
   )
  )

(defun demo-msc-nonconvex-2d
    (&key
     (use-msc? t)
     )
  (setq *config-task* (instance alpine2-config-task :init))
  (cond (use-msc?
         (setq *sqp-opt*
               (instance sqp-msc-optimization :init
                         :num-msc 16
                         :dispersion-scale 1e-1
                         :dispersion-scale-decrease-ratio 0.8
                         :config-task *config-task*
                         :post-process-func
                         #'(lambda (sqp)
                             (warning-message 3 "dispersion-scale: ~a~%"
                                              (send sqp :get-val '_dispersion-scale))
                             (write-configuration :sqp sqp)
                             )
                         ))
         (send *sqp-opt* :put :filename
               (ros::resolve-ros-path "package://eus_qp/optmotiongen/logs/demo-msc-nonconvex-2d/config-msc.csv"))
         (dolist (config-task (send *sqp-opt* :config-task-list))
           (send config-task :set-config
                 (v+ (send config-task :config-vector)
                     (gaussian-random 2)
                     ))
           )
         )
        (t
         (setq *sqp-opt*
               (instance sqp-optimization :init
                         :config-task *config-task*
                         :post-process-func
                         #'(lambda (sqp)
                             (write-configuration :sqp sqp)
                             )
                         ))
         (send *sqp-opt* :put :filename
               (ros::resolve-ros-path "package://eus_qp/optmotiongen/logs/demo-msc-nonconvex-2d/config.csv"))
         ))
  (unix:system
   (format nil "DIRNAME=`dirname ~a`; if [ ! -d $DIRNAME ]; then mkdir -p $DIRNAME; fi" (send *sqp-opt* :get :filename)))
  (write-configuration :sqp *sqp-opt* :initialize? t)
  (send *sqp-opt* :optimize
        :loop-num 100
        :print-status-interval 10
        )
  )
(warn "(demo-msc-nonconvex-2d)~%")
