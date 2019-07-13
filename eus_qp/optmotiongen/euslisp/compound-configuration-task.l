(defclass compound-configuration-task
  :super propertied-object
  :slots (;; config-task
          (_config-task-list "")
          ;; buffer
          (_task-jacobi "buffer for $\frac{\partial \bm{e}}{\partial \bm{q}}$")
          (_ineq-mat "buffer for $\bm{C}$")
          (_eq-mat "buffer for $\bm{A}$")
          (_regular-mat "buffer for $\bm{W}_{\mathit{reg}}$")
          )
  :documentation
  "
  "
  )


(defmethod compound-configuration-task
  (:init
   (&key
    (config-task-list)
    )
   (setq _config-task-list config-task-list)
   )
  (:dim-config
   ()
   (apply #'+ (send-all _config-task-list :dim-config))
   )
  (:config-vector
   ()
   (apply #'concatenate float-vector (send-all _config-task-list :config-vector))
   )
  (:set-config
   (config-new
    &key
    (relative? nil)
    )
   (dolist (config-task _config-task-list)
     (send config-task :set-config (subseq config-new 0 (send config-task :dim-config))
           :relative? relative?)
     (setq config-new (subseq config-new (send config-task :dim-config)))
     )
   )
  )

(defmethod compound-configuration-task
  (:task-value
   (&key (update? t))
   (apply #'concatenate float-vector (send-all _config-task-list :task-value :update? update?))
   )
  (:task-jacobian
   ()
   (cond (_task-jacobi
          (fill (array-entity _task-jacobi) 0)
          (let* ((block-mat) (i 0) (j 0))
            (dolist (config-task _config-task-list)
              (setq block-mat (send config-task :task-jacobian))
              (copy-block-matrix _task-jacobi block-mat i j)
              (setq i (+ i (length (matrix-column block-mat 0))))
              (setq j (+ j (length (matrix-row block-mat 0))))
              ))
          )
         (t
          (setq _task-jacobi
                (apply
                 #'concatenate-matrix-diagonal
                 (send-all _config-task-list :task-jacobian)
                 ))
          ))
   _task-jacobi
   )
  )

(defmethod compound-configuration-task
  (:config-inequality-constraint-matrix
   (&key (update? nil) (update-collision? nil))
   (cond (_ineq-mat
          (fill (array-entity _ineq-mat) 0)
          (let* ((block-mat) (i 0) (j 0))
            (dolist (config-task _config-task-list)
              (setq block-mat
                    (send config-task :config-inequality-constraint-matrix
                          :update? update? :update-collision? update-collision?))
              (copy-block-matrix _ineq-mat block-mat i j)
              (setq i (+ i (length (matrix-column block-mat 0))))
              (setq j (+ j (length (matrix-row block-mat 0))))
              ))
          )
         (t
          (setq _ineq-mat
                (apply
                 #'concatenate-matrix-diagonal
                 (send-all _config-task-list :config-inequality-constraint-matrix
                           :update? update? :update-collision? update-collision?)
                 ))
          ))
   _ineq-mat
   )
  (:config-inequality-constraint-vector
   (&key (update? t) (update-collision? nil))
   (apply
    #'concatenate
    float-vector
    (send-all _config-task-list :config-inequality-constraint-vector
              :update? update? :update-collision? update-collision?)
    ))
  (:config-equality-constraint-matrix
   (&key (update? nil))
   (cond (_eq-mat
          (fill (array-entity _eq-mat) 0)
          (let* ((block-mat) (i 0) (j 0))
            (dolist (config-task _config-task-list)
              (setq block-mat
                    (send config-task :config-equality-constraint-matrix
                          :update? update?))
              (copy-block-matrix _eq-mat block-mat i j)
              (setq i (+ i (length (matrix-column block-mat 0))))
              (setq j (+ j (length (matrix-row block-mat 0))))
              ))
          )
         (t
          (setq _eq-mat
                (apply
                 #'concatenate-matrix-diagonal
                 (send-all _config-task-list :config-equality-constraint-matrix
                           :update? update?)
                 ))
          ))
   _eq-mat
   )
  (:config-equality-constraint-vector
   (&key (update? t))
   (apply
    #'concatenate
    float-vector
    (send-all _config-task-list :config-equality-constraint-vector
              :update? update?)
    ))
  (:update-collision-inequality-constraint
   ()
   (dolist (config-task _config-task-list)
     (when (member :update-collision-inequality-constraint (send config-task :methods))
       (send config-task :update-collision-inequality-constraint))
     )
   )
  )

(defmethod compound-configuration-task
  (:regular-matrix
   ()
   (cond (_regular-mat
          (fill (array-entity _regular-mat) 0)
          (let* ((block-mat) (i 0) (j 0))
            (dolist (config-task _config-task-list)
              (setq block-mat (send config-task :regular-matrix))
              (copy-block-matrix _regular-mat block-mat i j)
              (setq i (+ i (length (matrix-column block-mat 0))))
              (setq j (+ j (length (matrix-row block-mat 0))))
              ))
          )
         (t
          (setq _regular-mat
                (apply
                 #'concatenate-matrix-diagonal
                 (send-all _config-task-list :regular-matrix)
                 ))
          ))
   _regular-mat
   )
  (:regular-vector
   ()
   (apply
    #'concatenate
    float-vector
    (send-all _config-task-list :regular-vector)
    ))
  )

(defmethod compound-configuration-task
  (:update-viewer
   ()
   (send-all _config-task-list :update-viewer)
   )
  (:print-status
   ()
   (send-all _config-task-list :print-status)
   )
  (:print-setting-information
   ()
   (dolist (config-task _config-task-list)
     (when (member :print-setting-information (send config-task :methods))
       (send config-task :print-setting-information))
     )
   )
  )
