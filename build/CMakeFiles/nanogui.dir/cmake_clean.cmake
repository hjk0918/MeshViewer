file(REMOVE_RECURSE
  "libnanogui.dll"
  "libnanogui.dll.a"
  "libnanogui.dll.manifest"
  "libnanogui.pdb"
)

# Per-language clean rules from dependency scanning.
foreach(lang C CXX)
  include(CMakeFiles/nanogui.dir/cmake_clean_${lang}.cmake OPTIONAL)
endforeach()
