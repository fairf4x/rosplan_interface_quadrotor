(define (problem rosplan_quad_task)
(:domain rosplan_quad)
(:objects
    q1 - quad
)
(:init
    (grounded q1)
)
(:goal (and
    (finished q1)
    (squaredone q1)
)))
