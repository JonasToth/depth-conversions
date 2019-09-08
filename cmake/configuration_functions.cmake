function(common_target_properties target_name)
    set_target_properties(${target_name}
        PROPERTIES
        CXX_STANDARD 17
        CXX_EXTENSIONS OFF
        CXX_STANDARD_REQUIRED ON
        )
    target_compile_definitions(${target_name}
        PUBLIC
        "GSL_THROW_ON_CONTRACT_VIOLATION=1")
endfunction()
