(load "package://eus_qp/optmotiongen/euslisp/sqp-msc-optimization.l")
(load "package://eus_qp/optmotiongen/euslisp/validate-jacobian.l")


;;;;;;;;;;;;;;;;;;;;;;;;;
;; demo-msc-2d-config-task
;;;;;;;;;;;;;;;;;;;;;;;;;

(defclass demo-msc-2d-config-task
  :super propertied-object
  :slots (_config
          _dim-config
          _delta-config
          _norm-regular-scale-max
          _norm-regular-scale-offset
          _norm-regular-scale-coeff
          _sqp-opt
          )
  )

(defmethod demo-msc-2d-config-task
  (:init
   (&key
    (initial-config (float-vector 0 0))
    (norm-regular-scale-coeff 1.0)
    (norm-regular-scale-max 1e10)
    (norm-regular-scale-offset 1e-10)
    (config-min (float-vector -1e10 -1e10))
    (config-max (float-vector 1e10 1e10))
    (delta-config)
    )
   (setq _config initial-config)
   (setq _dim-config 2)
   (setq _norm-regular-scale-coeff norm-regular-scale-coeff)
   (setq _norm-regular-scale-max norm-regular-scale-max)
   (setq _norm-regular-scale-offset norm-regular-scale-offset)
   (setq _config-min config-min)
   (setq _config-max config-max)
   (setq _delta-config delta-config)
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
    )
   (warning-message 1 ":task-value should be implemented in subclass.~%")
   )
  (:task-jacobian
   ()
   (warning-message 1 ":task-jacobian should be implemented in subclass.~%")
   )
  (:config-inequality-constraint-matrix
   (&key
    (update? nil)
    )
   (when (or update? (not (send self :get :config-inequality-constraint-matrix)))
     (send self :put :config-inequality-constraint-matrix
           (apply
            #'concatenate-matrix-column
            (remove
             nil
             (list
              (unit-matrix _dim-config)
              (scale-matrix -1 (unit-matrix _dim-config))
              (if _delta-config
                  (unit-matrix _dim-config))
              (if _delta-config
                  (scale-matrix -1 (unit-matrix _dim-config)))
              )))
           ))
   (send self :get :config-inequality-constraint-matrix)
   )
  (:config-inequality-constraint-vector
   (&key
    (update? t)
    )
   (when (or update? (not (send self :get :config-inequality-constraint-vector)))
     (send self :put :config-inequality-constraint-vector
           (apply
            #'concatenate
            float-vector
            (remove
             nil
             (list
              (v- _config-min _config)
              (v- _config _config-max)
              (if _delta-config
                  (scale -1 _delta-config))
              (if _delta-config
                  (scale -1 _delta-config))
              )))
           ))
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
  (:demo
   (&key
    (use-msc? t)
    (loop-num 100)
    (num-msc 4)
    (dispersion-scale 1e3)
    (dispersion-scale-decrease-ratio 0.9)
    (initial-noise-variance 1e-6)
    (log-name (if use-msc? "config-msc.csv" "config.csv"))
    (log-dir (ros::resolve-ros-path "package://eus_qp/optmotiongen/logs/demo-msc-nonconvex-2d/"))
    )
   (cond (use-msc?
          (setq _sqp-opt
                (instance sqp-msc-optimization :init
                          :num-msc num-msc
                          :dispersion-scale dispersion-scale
                          :dispersion-scale-decrease-ratio dispersion-scale-decrease-ratio
                          :config-task self
                          :post-process-func
                          #'(lambda (sqp)
                              (warning-message 3 "dispersion-scale: ~a~%"
                                               (send sqp :get-val '_dispersion-scale))
                              (write-configuration :sqp sqp)
                              )
                          ))
          (dolist (config-task (send _sqp-opt :config-task-list))
            (send config-task :set-config
                  (v+ (send config-task :config-vector)
                      (gaussian-random 2 0 (sqrt initial-noise-variance))
                      ))
            )
          )
         (t
          (setq _sqp-opt
                (instance sqp-optimization :init
                          :config-task *config-task*
                          :post-process-func
                          #'(lambda (sqp)
                              ;; (validate-jacobian :config-task (send sqp :config-task))
                              (write-configuration :sqp sqp)
                              )
                          ))
          ))
   (send _sqp-opt :put :filename (format nil "~a/~a" log-dir log-name))
   (unix:system
    (format nil "DIRNAME=`dirname ~a`; if [ ! -d $DIRNAME ]; then mkdir -p $DIRNAME; fi" (send _sqp-opt :get :filename)))
   (write-configuration :sqp _sqp-opt :initialize? t)
   (send _sqp-opt :optimize
         :loop-num loop-num
         :print-status-interval 10
         )
   )
  )


;;;;;;;;;;;;;;;;;;;;;;;;;
;; alpine2
;; http://benchmarkfcns.xyz/benchmarkfcns/alpinen2fcn.html
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
  :super demo-msc-2d-config-task
  :slots (_objective-scale
          )
  )

(defmethod alpine2-config-task
  (:init
   (&key
    )
   (send-super
    :init
    :initial-config (float-vector 7.0 4.0)
    :norm-regular-scale-coeff 1.0
    :norm-regular-scale-max 1e10
    :norm-regular-scale-offset 1e-6
    :config-min (float-vector 0 0)
    :config-max (float-vector 10 10)
    :delta-config nil
    )
   (setq _objective-scale 20)
   )
  (:task-value
   (&key
    (update?)
    )
   (float-vector
    (* _objective-scale (- 8.0 (alpine2 (elt _config 0) (elt _config 1)))))
   )
  (:task-jacobian
   ()
   (matrix
    (scale
     (* -1 _objective-scale)
     (alpine2-jacobi (elt _config 0) (elt _config 1))
     ))
   )
  (:demo
   (&key
    (use-msc? t)
    )
   (send-super
    :demo
    :use-msc? use-msc?
    :loop-num 100
    :num-msc 10
    :dispersion-scale 1e3
    :dispersion-scale-decrease-ratio 0.8
    :initial-noise-variance 1e-5
    )
   )
  )

;;;;;;;;;;;;;;;;;;;;;;;;;
;; egg-crate
;; http://benchmarkfcns.xyz/benchmarkfcns/eggcratefcn.html
;;;;;;;;;;;;;;;;;;;;;;;;;

(defun egg-crate
    (x y &aux (sin-x (sin x)) (sin-y (sin y)))
  (+ (* x x) (* y y) (* 25 (+ (* sin-x sin-x) (* sin-y sin-y))))
  )

(defun egg-crate-jacobi
    (x y &aux (sin-x (sin x)) (sin-y (sin y)) (cos-x (cos x)) (cos-y (cos y)))
  (float-vector
   (+ (* 2 x) (* 25 (* 2 sin-x cos-x)))
   (+ (* 2 y) (* 25 (* 2 sin-y cos-y)))
   )
  )

(defclass egg-crate-config-task
  :super demo-msc-2d-config-task
  :slots (_objective-scale
          )
  )

(defmethod egg-crate-config-task
  (:init
   (&key
    )
   (send-super
    :init
    :initial-config (float-vector 2.5 3.5)
    :norm-regular-scale-coeff 1.0
    :norm-regular-scale-max 1e10
    :norm-regular-scale-offset 1e-6
    :config-min (float-vector -5 -5)
    :config-max (float-vector 5 5)
    ;; :delta-config (float-vector 1.0 1.0)
    )
   (setq _objective-scale 5)
   )
  (:task-value
   (&key
    (update?)
    )
   (float-vector
    (* _objective-scale (egg-crate (elt _config 0) (elt _config 1))))
   )
  (:task-jacobian
   ()
   (matrix
    (scale
     _objective-scale
     (egg-crate-jacobi (elt _config 0) (elt _config 1))))
   )
  (:demo
   (&key
    (use-msc? t)
    )
   (send-super
    :demo
    :use-msc? use-msc?
    :loop-num 100
    :num-msc 20
    :dispersion-scale 1e3
    :dispersion-scale-decrease-ratio 0.5
    :initial-noise-variance 1e-5
    )
   )
  )


(defun write-configuration
    (&key
     (sqp)
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
   (dotimes (i (length config-list))
     (format f "~10,5f" (elt config-list i))
     (unless (= i (1- (length config-list)))
       (format f ", "))
     )
   (format f "~%")
   )
  )

(defun demo-msc-nonconvex-2d
    (&key
     (use-msc? t)
     ;; (config-task-class alpine2-config-task)
     (config-task-class egg-crate-config-task)
     )
  (setq *config-task* (instance config-task-class :init))
  (send *config-task* :demo :use-msc? use-msc?)
  )
(warn "(demo-msc-nonconvex-2d)~%")
