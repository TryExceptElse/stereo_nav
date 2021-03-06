

cdef extern from 'third_party/pipe.h':
    cdef:
        struct pipe_t
        struct pipe_producer_t
        struct pipe_consumer_t
        struct pipe_generic_t

    pipe_t*             pipe_new            (size_t elem_size,
                                             size_t limit) nogil
    pipe_producer_t*    pipe_producer_new   (pipe_t*) nogil
    pipe_consumer_t*    pipe_consumer_new   (pipe_t*) nogil

    void                pipe_free           (pipe_t*) nogil
    void                pipe_producer_free  (pipe_producer_t*) nogil
    void                pipe_consumer_free  (pipe_consumer_t*) nogil

    void                pipe_push           (pipe_producer_t*,
                                             const void* elems,
                                             size_t count) nogil

    size_t              pipe_pop            (pipe_consumer_t*,
                                             void* target,
                                             size_t count) nogil
