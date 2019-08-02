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
