file(REMOVE_RECURSE
  "glfw3.dll"
  "glfw3.dll.manifest"
  "glfw3.pdb"
  "libglfw3dll.a"
)

# Per-language clean rules from dependency scanning.
foreach(lang C)
  include(CMakeFiles/glfw.dir/cmake_clean_${lang}.cmake OPTIONAL)
endforeach()
