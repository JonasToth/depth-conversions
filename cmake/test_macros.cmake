if (WITH_VALGRIND)
    set(test_executor "${MEMORYCHECK_COMMAND} --tool=memcheck
                                              --leak-check=yes
                                              --show-reachable=yes
                                              --num-callers=20
                                              --track-fds=yes")

else ()
    set (test_executor "")
endif (WITH_VALGRIND)

