import os
import sys

def patch_file(file_path):
    with open(file_path, 'r') as f:
        content = f.read()

    # Define what to insert
    policy_check = """
  if (CMAKE_POLICY_VERSION_MINIMUM)
    message(STATUS "DEBUG: CMAKE_POLICY_VERSION_MINIMUM inside Arrow is: ${CMAKE_POLICY_VERSION_MINIMUM}")
    set(MIMALLOC_POLICY_ARGS
        -DCMAKE_POLICY_VERSION_MINIMUM=${CMAKE_POLICY_VERSION_MINIMUM})
  else()
    set(MIMALLOC_POLICY_ARGS "")
  endif()

"""
    
    # Insert policy check before MIMALLOC_CMAKE_ARGS definition
    target_str = "set(MIMALLOC_CMAKE_ARGS"
    if target_str in content and "MIMALLOC_POLICY_ARGS" not in content:
        content = content.replace(target_str, policy_check + target_str)
        print("Inserted policy check logic.")
    else:
        print("Policy check logic already present or target not found.")

    # Insert MIMALLOC_POLICY_ARGS usage
    target_usage = "-DMI_BUILD_TESTS=OFF)"
    replacement_usage = "-DMI_BUILD_TESTS=OFF\n      ${MIMALLOC_POLICY_ARGS})"
    
    if target_usage in content:
        content = content.replace(target_usage, replacement_usage)
        print("Inserted policy args usage.")
    else:
        print("Target usage string not found (maybe already patched).")

    with open(file_path, 'w') as f:
        f.write(content)

if __name__ == "__main__":
    if len(sys.argv) < 2:
        print("Usage: python patch_mimalloc.py <path_to_ThirdpartyToolchain.cmake>")
        sys.exit(1)
    
    patch_file(sys.argv[1])
