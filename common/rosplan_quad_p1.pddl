(define (problem p1)
    (:domain rosplan_quad)
    (:objects q1 - quad)
    (:init
        (grounded q1)
    )
    
    (:goal
        (and
            (finished q1)
            (squaredone q1)
        )
    )
)
