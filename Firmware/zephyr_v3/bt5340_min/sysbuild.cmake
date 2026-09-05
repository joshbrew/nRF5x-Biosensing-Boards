# Keep every image's compiler capability cache in the generated build tree.
# ExternalProject owns this CMake argument property; no SDK files are modified.
get_property(bio_images GLOBAL PROPERTY sysbuild_images)
foreach(bio_image ${bio_images})
  set_property(TARGET ${bio_image} APPEND PROPERTY _EP_CMAKE_ARGS
    "-DUSER_CACHE_DIR:PATH=${CMAKE_BINARY_DIR}/.cache/${bio_image}")
endforeach()
