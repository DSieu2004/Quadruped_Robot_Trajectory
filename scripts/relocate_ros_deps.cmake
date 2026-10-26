# Some Humble binary exports embed absolute paths to other ROS packages.
# Relocate only paths actually provided by the local dependency overlay.
if(NOT IS_DIRECTORY "${GO2_DEPS_ROOT}/opt/ros/humble")
  message(FATAL_ERROR "Set GO2_DEPS_ROOT to the extracted dependency root")
endif()
file(GLOB_RECURSE exports "${GO2_DEPS_ROOT}/opt/ros/humble/*.cmake")
foreach(export IN LISTS exports)
  file(READ "${export}" content)
  string(REGEX MATCHALL "/opt/ros/humble/[A-Za-z0-9_./+-]+" paths "${content}")
  list(REMOVE_DUPLICATES paths)
  foreach(path IN LISTS paths)
    if(EXISTS "${GO2_DEPS_ROOT}${path}")
      # Boundaries make this idempotent and avoid replacing a suffix of an
      # already-relocated absolute path or a parent of a longer include path.
      string(REPLACE "\"${path}\"" "\"${GO2_DEPS_ROOT}${path}\"" content "${content}")
      string(REPLACE "\"${path};" "\"${GO2_DEPS_ROOT}${path};" content "${content}")
      string(REPLACE ";${path};" ";${GO2_DEPS_ROOT}${path};" content "${content}")
      string(REPLACE ";${path}\"" ";${GO2_DEPS_ROOT}${path}\"" content "${content}")
    endif()
  endforeach()
  file(WRITE "${export}" "${content}")
endforeach()
