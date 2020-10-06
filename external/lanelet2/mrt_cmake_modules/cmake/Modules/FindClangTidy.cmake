find_program(
    ClangTidy_EXE
    NAMES "clang-tidy"
    DOC "Path to clang-tidy executable")

include(FindPackageHandleStandardArgs)
find_package_handle_standard_args(
    ClangTidy
    FOUND_VAR ClangTidy_FOUND
    REQUIRED_VARS ClangTidy_EXE)
mark_as_advanced(CLANG_TIDY_EXE)
