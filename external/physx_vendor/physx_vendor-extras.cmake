# Exported by find_package(physx_vendor). Points consumers at the PhysX SDK
# tree installed by this vendor package (<prefix>/opt/physx), preserving the
# upstream SDK layout: include/, bin/linux.x86_64/release/, snippets/.
#
# physx_vendor_DIR is <prefix>/share/physx_vendor/cmake, so the install prefix
# is three levels up.
get_filename_component(_physx_vendor_prefix "${physx_vendor_DIR}/../../.." ABSOLUTE)

if(NOT PHYSX_SDK_DIR)
  set(PHYSX_SDK_DIR "${_physx_vendor_prefix}/opt/physx"
      CACHE PATH "PhysX SDK root (provided by physx_vendor)")
endif()
if(NOT PHYSX_LIB_DIR)
  set(PHYSX_LIB_DIR "${PHYSX_SDK_DIR}/bin/linux.x86_64/release"
      CACHE PATH "PhysX static libraries (provided by physx_vendor)")
endif()

unset(_physx_vendor_prefix)
